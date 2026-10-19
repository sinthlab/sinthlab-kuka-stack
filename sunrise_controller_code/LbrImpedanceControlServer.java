package lbr_fri_ros2;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.util.Locale;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.IApplicationUI;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.connectivity.fastRobotInterface.ClientCommandMode;
import com.kuka.connectivity.fastRobotInterface.FRIChannelInformation;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.fastRobotInterface.IFRISessionListener;
import com.kuka.connectivity.fastRobotInterface.FRIJointOverlay;

/**
 * LbrImpedanceControlServer
 * 
 * Hardware-native Cartesian Impedance Control application for the KUKA Sunrise cabinet.
 * This runs pure Cartesian Impedance locally at 1000Hz while listening to joint
 * position targets via FRI from the ROS 2 driver.
 *
 * Start-up order (run()):
 *   1. ALWAYS, with no prompt: joint PTP to MECHANICAL ZERO, then to the READY pose, in plain
 *      POSITION control. Whatever posture the arm was left in, every experiment then starts its
 *      move_to_start from the same well-conditioned pose. Starting from an arbitrary posture can put
 *      the arm in or near a singular configuration, where the Cartesian impedance spring cannot act
 *      in some directions, so ROS joint targets are tracked badly or the arm locks up. The cabinet's
 *      own position controller has no such problem, so this is done BEFORE impedance is switched on.
 *   2. Open FRI and wait up to 60 s for the ROS 2 client. This comes after the PTPs, so the first
 *      state ROS reads -- and the start of its first move -- is the ready pose.
 *   3. Hold the pose in Cartesian impedance with the FRI overlay; ROS moves the equilibrium.
 *
 * CANCEL: every set-up question has a "Cancel" choice, which ends the app before anything moves. Once
 * running, the app is ended with the SmartPAD's own stop. Neither is a safety function: the E-stop is
 * the E-stop.
 *
 * SETTINGS REPORT: once the selections are made, the app sends them to the ROS computer (the remote
 * IP chosen) once a second, as one JSON line over UDP to SETTINGS_PORT, until it ends. ROS records
 * them once per run (smartpad_params.json in the run's folder) and checks them against the experiment
 * (sinthlab_bringup/helpers/smartpad_link.py). Repeated only so the report gets through. Best effort: if the network drops the packets, the
 * app runs exactly as before and ROS reports that it has heard nothing.
 *
 * NEURAL SYNC: TTL pulses on the EL2124 outputs 1-4 (TtlPulser), for the Blackrock NSP. Call
 * sendTtlPulse(output, widthMs) from this app, or send "PULSE <output> <width ms>" over UDP to
 * TtlPulser.DEFAULT_PORT (30002) from the ROS computer. Until the NspSync I/O configuration is in the
 * project the outputs are simply unavailable (logged once) and the app runs as before. Test the
 * outputs on their own with the PulseTestServer app. README, "Neural synchronisation".
 */
public class LbrImpedanceControlServer extends RoboticsAPIApplication {
    // Injectable dependencies
    @Inject
    private LBR lbr_;
    @Inject
    private IApplicationUI applicationUi;

    // End-effector tool template name — MUST match the tool defined in RoboticsAPI.data.xml.
    // Attaching it lets the cabinet gravity-compensate the payload so compliant control modes can
    // activate (otherwise the EE weight reads as external torque and StateGuard aborts).
    // IMPORTANT: the tool's loadData (in RoboticsAPI.data.xml) MUST match what is physically mounted:
    //   - bare flange -> loadData mass = 0  (attaching a zero-load tool is a safe no-op)
    //   - EE mounted  -> fill mass + COM (SmartPad "Determine", or enter manually) and re-sync.
    // The Java does not change between those cases — only the tool's loadData does.
    private static final String EE_TOOL_TEMPLATE = "SinthLabIiwa7EE";
    private Tool ee_tool_;

    // Neural-sync TTL outputs (see the class doc). Never null after initialize().
    private TtlPulser ttl_;

    // Settings report to ROS -- must equal SETTINGS_PORT in sinthlab_bringup/helpers/smartpad_link.py.
    private static final int SETTINGS_PORT = 30001;
    private static final long SETTINGS_PERIOD_MS = 1000;
    private volatile boolean reporting_ = false;
    private String profile_name_ = "";
    private double ns_damping_used_ = 0.0;

    // Cancel: the last choice of every set-up question; ends the app before anything moves.
    private static final String CANCEL = "Cancel";
    private boolean cancelled_ = false;

    // FRI Networking Parameters
    private String client_name_;
    private String[] client_names_ = { "172.31.1.148", "192.170.10.100" };
    private int send_period_;
    private String[] send_periods_ = { "1", "2", "5", "10" };

    private FRIConfiguration fri_configuration_;
    private FRISession fri_session_;
    private FRIJointOverlay fri_overlay_;
    private CartesianImpedanceControlMode control_mode_;

    // COMPLIANCE PARAMETERS
    // Per-axis Cartesian stiffness:  X,Y,Z in [N/m],  A,B,C in [Nm/rad] -- the chosen profile, below
    private double[] K;
    // Per-axis damping ratio [0.1 .. 1.0]
    private double[] D = { 0.7, 0.7, 0.7, 0.7, 0.7, 0.7 };
    private double ns_stiffness = 30.0;
    private double ns_damping = 0.7;

    // KUKA Sunrise valid ranges; every value is clamped to these (trans <= 5000 N/m, rot <= 300 Nm/rad)
    private static final double[] K_MAX = { 5000.0, 5000.0, 5000.0, 300.0, 300.0, 300.0 };
    private static final double NS_DAMPING_MIN = 0.3;

    // UI Options for Compliance. Each profile is a full per-axis {X,Y,Z,A,B,C} stiffness diagonal
    // (X,Y,Z [N/m], A,B,C [Nm/rad]), named by its values, not by an experiment. The first is listed
    // first on the SmartPad. Which experiment uses which: experiment_ctrl_gui/experiments.py (the
    // dashboard's SmartPad checklist) and the README.
    private String[] stiffness_profiles_ = {
        "Uniform 100",
        "Uniform 400",
        "Uniform 1000",
        "X lock 2500, Y/Z 1000, rot 120"
    };
    private double[][] stiffness_vals_ = {
        // Apple pluck. The resistance slider softens it further from ROS (50 N/m up to this base).
        {  100.0,  100.0,  100.0, 300.0, 300.0, 300.0 },
        // Apple pluck with a stiffer base: the resistance slider then spans 50-400 N/m.
        {  400.0,  400.0,  400.0, 300.0, 300.0, 300.0 },
        // Restricted plane (the sine rail) and Free Move: firm everywhere, so the arm follows its
        // moving equilibrium closely; the fixture or the admittance decides where that is.
        { 1000.0, 1000.0, 1000.0, 300.0, 300.0, 300.0 },
        // Maze and the rail pre-training tasks (vertical Y-Z plane, restricted_axis "x"):
        //   X 2500 locks the radial axis, so the plane is enforced by the CABINET, hard.
        //   Y/Z 1000 = firm rail walls (~10 N per cm off the rail). Along a rail the fixture's
        //   projection returns the measured pose, so the spring is slack there whatever these are.
        //   rot 120: orientation stiffness defines nothing about the maze (its constraints are all
        //   translational), but holding orientation while translating is the expensive motion, so a
        //   lower rot makes guiding lighter without softening the plane or the rails. The price is a
        //   little more tool twist -- watch the apple angle.
        { 2500.0, 1000.0, 1000.0, 120.0, 120.0, 120.0 }
    };

    // Start-up sequence, every run: MECHANICAL ZERO (a known, repeatable posture whatever the arm was
    // left in), then the READY POSE -- the restricted-plane start: well conditioned (smallest singular
    // value of the Jacobian 0.206, against 0.0 when straight), and every experiment's start is
    // reachable from it. Degrees, A1..A7.
    private static final double[] ZERO_POSE_DEG = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    private static final double[] READY_POSE_DEG = { 0.0, 10.0, 0.0, -80.0, 0.0, 90.0, 0.0 };
    private static final double READY_VEL_REL = 0.2;   // PTP speed, fraction of the joint limits

    // Damping ratio. The cabinet damps the flange's actual velocity, so along a rail it is felt as drag
    // that grows with speed; lower = lighter to move, but walls and returns may ring. 0.1 is Sunrise's
    // minimum.
    private String[] damping_options_ = { "0.1 (Very light)", "0.2 (Light)", "0.3 (Underdamped)",
                                          "0.7 (Standard)", "1.0 (Critically Damped)" };
    private double[] damping_vals_ = { 0.1, 0.2, 0.3, 0.7, 1.0 };

    // Null-space (elbow) stiffness [Nm/rad]: how firmly the cabinet holds the arm's redundant elbow
    // posture. Translating the flange moves the elbow too, so this is felt as a constant resistance.
    // Lower = lighter; the elbow is then held less firmly, so watch it does not swing. No 0: an unheld
    // elbow drifts under the gravity-compensation residual.
    private String[] ns_stiffness_options_ = { "30 (Standard)", "15 (Light)", "5 (Very light)" };
    private double[] ns_stiffness_vals_ = { 30.0, 15.0, 5.0 };

    /**
     * One SmartPAD selection dialog, with "Cancel" as its last choice. Returns the chosen index, or -1
     * for Cancel (which also requests the app to end).
     */
    private int ask(String question, String[] options) {
        String[] withCancel = new String[options.length + 1];
        System.arraycopy(options, 0, withCancel, 0, options.length);
        withCancel[options.length] = CANCEL;
        int i = applicationUi.displayModalDialog(ApplicationDialogType.QUESTION, question, withCancel);
        if (i == options.length) {
            cancelled_ = true;
            getLogger().info("Cancelled from the set-up questions.");
            return -1;
        }
        return i;
    }

    /**
     * Prompts the user on the SmartPAD to configure the connection. Returns false if Cancel was chosen.
     */
    public boolean request_user_config() {
        // Ask for Send Period
        int selectedButtonIndex = ask("Select the desired FRI send period [ms]:", send_periods_);
        if (selectedButtonIndex < 0) return false;
        send_period_ = Integer.valueOf(send_periods_[selectedButtonIndex]);
        getLogger().info("Send period set to: " + send_period_);

        // Ask for Remote IP
        selectedButtonIndex = ask("Select your remote IP address:", client_names_);
        if (selectedButtonIndex < 0) return false;
        client_name_ = client_names_[selectedButtonIndex];
        getLogger().info("Remote address set to: " + client_name_);

        // Ask for Stiffness Profile
        selectedButtonIndex = ask("Select Cartesian Stiffness (K diagonal):", stiffness_profiles_);
        if (selectedButtonIndex < 0) return false;
        K = stiffness_vals_[selectedButtonIndex];
        profile_name_ = stiffness_profiles_[selectedButtonIndex];
        getLogger().info("Stiffness Profile set to: " + stiffness_profiles_[selectedButtonIndex]);

        // Ask for Damping Ratio
        selectedButtonIndex = ask("Select Damping Ratio (D0):", damping_options_);
        if (selectedButtonIndex < 0) return false;
        double d0 = damping_vals_[selectedButtonIndex];
        for (int i = 0; i < 6; i++) {
            D[i] = d0;
        }
        ns_damping = d0;
        getLogger().info("Damping Ratio set to: " + d0);

        // Ask for Null-space (elbow) Stiffness
        selectedButtonIndex = ask("Select Null-space (elbow) Stiffness [Nm/rad]:", ns_stiffness_options_);
        if (selectedButtonIndex < 0) return false;
        ns_stiffness = ns_stiffness_vals_[selectedButtonIndex];
        getLogger().info("Null-space stiffness set to: " + ns_stiffness);

        // Setup the Cartesian Impedance Control Mode with per-axis stiffness & damping
        control_mode_ = new CartesianImpedanceControlMode();
        applyCartesianImpedance(control_mode_, K, D);

        getLogger().info("Control mode set to: Cartesian Impedance Control");
        getLogger().info("Stiffness (X, Y, Z, A, B, C): " + K[0] + ", " + K[1] + ", " + K[2] + 
                         ", " + K[3] + ", " + K[4] + ", " + K[5]);
        ns_damping_used_ = Math.max(NS_DAMPING_MIN, Math.min(ns_damping, 1.0));
        getLogger().info("Damping " + d0 + " (elbow damping " + ns_damping_used_
                         + "), null-space stiffness " + ns_stiffness);
        return true;
    }

    /**
     * Applies per-axis Cartesian stiffness and damping to the impedance control mode, clamping
     * every value to the Sunrise-valid range. Centralising this lets the stiffness profiles be
     * anisotropic (e.g. stiff perpendicular to a plane, soft along it) so the cabinet itself can
     * act as a hardware virtual fixture.
     */
    private void applyCartesianImpedance(CartesianImpedanceControlMode mode, double[] k, double[] d) {
        CartDOF[] dof = { CartDOF.X, CartDOF.Y, CartDOF.Z, CartDOF.A, CartDOF.B, CartDOF.C };
        for (int i = 0; i < 6; i++) {
            double ki = Math.max(0.0, Math.min(k[i], K_MAX[i]));
            double di = Math.max(0.1, Math.min(d[i], 1.0));
            mode.parametrize(dof[i]).setStiffness(ki);
            mode.parametrize(dof[i]).setDamping(di);
        }
        mode.setNullSpaceStiffness(ns_stiffness);
        // Null-space damping has a narrower valid range than the Cartesian damping (0.3 .. 1.0 against
        // 0.1 .. 1.0), so a Cartesian damping of 0.1 or 0.2 keeps the elbow damping at 0.3.
        mode.setNullSpaceDamping(Math.max(NS_DAMPING_MIN, Math.min(ns_damping, 1.0)));
    }

    /**
     * Mechanical zero, then the ready pose, both as joint PTPs in position control. Always runs, with
     * no prompt -- see the class doc for why.
     */
    private void moveToReadyViaZero() {
        getLogger().info("Moving to mechanical zero in position control...");
        ptpDeg(ZERO_POSE_DEG);
        getLogger().info("Moving to the ready pose [0, 10, 0, -80, 0, 90, 0] in position control...");
        ptpDeg(READY_POSE_DEG);
        getLogger().info("At the ready pose.");
    }

    private void ptpDeg(double[] deg) {
        double[] rad = new double[deg.length];
        for (int i = 0; i < rad.length; i++) {
            rad[i] = Math.toRadians(deg[i]);
        }
        lbr_.move(ptp(new JointPosition(rad)).setJointVelocityRel(READY_VEL_REL));
    }

    /**
     * Send the selections to the ROS computer once a second until the app ends (see the class doc).
     * One JSON line per packet; a failed send is logged once and the app carries on.
     */
    private void startSettingsReport() {
        final String json = String.format(Locale.ROOT,
            "{\"app\": \"LbrImpedanceControlServer\", \"stiffness_profile\": \"%s\", "
            + "\"stiffness\": [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f], \"damping\": %.2f, "
            + "\"nullspace_stiffness\": %.1f, \"nullspace_damping\": %.2f, \"send_period_ms\": %d, "
            + "\"tool\": \"%s\"}",
            profile_name_, K[0], K[1], K[2], K[3], K[4], K[5], D[0], ns_stiffness, ns_damping_used_,
            send_period_, EE_TOOL_TEMPLATE);
        reporting_ = true;
        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                DatagramSocket socket = null;
                boolean warned = false;
                try {
                    socket = new DatagramSocket();
                    byte[] data = json.getBytes("UTF-8");
                    DatagramPacket packet = new DatagramPacket(
                        data, data.length, InetAddress.getByName(client_name_), SETTINGS_PORT);
                    while (reporting_) {
                        try {
                            socket.send(packet);
                        } catch (Exception e) {
                            if (!warned) {
                                warned = true;
                                getLogger().warn("Settings report to " + client_name_ + ":" + SETTINGS_PORT
                                                 + " failed (" + e + "); the app carries on without it.");
                            }
                        }
                        Thread.sleep(SETTINGS_PERIOD_MS);
                    }
                } catch (Exception e) {
                    getLogger().warn("Settings report stopped: " + e);
                } finally {
                    if (socket != null) {
                        socket.close();
                    }
                }
            }
        }, "settings-report");
        t.setDaemon(true);
        t.start();
        getLogger().info("Reporting the SmartPad settings to " + client_name_ + ":" + SETTINGS_PORT + " every "
                         + SETTINGS_PERIOD_MS + " ms.");
    }

    /**
     * TTL pulse on EL2124 output `output` (1-4) for `widthMs`, for the NSP. Returns at once; false if
     * the outputs are unavailable or the arguments are out of range (TtlPulser).
     */
    public boolean sendTtlPulse(int output, int widthMs) {
        return ttl_ != null && ttl_.pulse(output, widthMs);
    }

    /** Open the TTL outputs and their UDP command port. Optional: the app runs the same without them. */
    private void startTtlOutputs() {
        ttl_ = TtlPulser.create(lbr_.getController());
        if (!ttl_.isAvailable()) {
            getLogger().info("Neural-sync TTL outputs unavailable (" + ttl_.reason() + ").");
            return;
        }
        if (ttl_.listen(TtlPulser.DEFAULT_PORT)) {
            getLogger().info("Neural-sync TTL outputs ready; pulse commands on UDP " + TtlPulser.DEFAULT_PORT + ".");
        } else {
            getLogger().warn("Neural-sync TTL outputs ready, but UDP " + TtlPulser.DEFAULT_PORT
                             + " is busy: only sendTtlPulse() in this app can pulse them.");
        }
    }

    /**
     * Initializes the Fast Robot Interface connection. Returns false if ROS did not connect within 60 s.
     */
    public boolean configure_fri() {
        fri_configuration_ = FRIConfiguration.createRemoteConfiguration(lbr_, client_name_);
        fri_configuration_.setSendPeriodMilliSec(send_period_);

        getLogger().info("Creating FRI connection to " + fri_configuration_.getHostName());

        fri_session_ = new FRISession(fri_configuration_);
        
        // We configure FRI to accept POSITION commands from ROS 2.
        // Because the robot is in Cartesian Impedance Mode, these positions act
        // as the "target equilibrium" resting point of the virtual springs, not rigid setpoints.
        fri_overlay_ = new FRIJointOverlay(fri_session_, ClientCommandMode.POSITION);

        fri_session_.addFRISessionListener(new IFRISessionListener() {
            @Override
            public void onFRISessionStateChanged(FRIChannelInformation info) {
                getLogger().info("Session State change " + info.getFRISessionState().toString());
            }

            @Override
            public void onFRIConnectionQualityChanged(FRIChannelInformation info) {
                getLogger().info("Quality change signalled " + info.getQuality());
                getLogger().info("Jitter " + info.getJitter());
                getLogger().info("Latency " + info.getLatency());
            }
        });

        // Block and wait for ROS 2 node to spin up and connect
        try {
            fri_session_.await(60, TimeUnit.SECONDS);
        } catch (final TimeoutException e) {
            getLogger().error("Connection timeout: Could not find ROS 2 client after 60 seconds.");
            return false;
        }

        getLogger().info("FRI connection established.");
        return true;
    }

    @Override
    public void initialize() {
        // Attach the end-effector tool so the controller accounts for its load in gravity
        // compensation. With loadData=0 in the template this is equivalent to a bare flange (safe);
        // once the EE is mounted and its loadData is filled + re-synced, the same call compensates it.
        ee_tool_ = getApplicationData().createFromTemplate(EE_TOOL_TEMPLATE);
        ee_tool_.attachTo(lbr_.getFlange());
        getLogger().info("Attached tool template '" + EE_TOOL_TEMPLATE + "' to the flange.");
        startTtlOutputs();

        if (request_user_config()) {
            startSettingsReport();
        }
    }

    @Override
    public void run() {
        if (cancelled_) {
            return;                 // Cancel chosen in a set-up question
        }
        // Zero, then ready, in position control BEFORE ROS connects -- see the class doc.
        moveToReadyViaZero();
        if (!configure_fri()) {
            return;                 // no ROS client: end rather than hold with nobody commanding
        }

        // Execute the motion holding command. The overlay lets ROS 2 update the target dynamically.
        lbr_.getFlange().move(
            positionHold(control_mode_, -1, TimeUnit.SECONDS)
            .addMotionOverlay(fri_overlay_)
        );
    }
    
    @Override
    public void dispose() {
        reporting_ = false;
        if (ttl_ != null) {
            ttl_.close();           // every output low
        }
        if (fri_session_ != null) {
            getLogger().info("Disposing FRI session.");
            fri_session_.close();
        }
        super.dispose();
    }
}