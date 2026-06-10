package lbr_fri_ros2;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
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
    // Per-axis Cartesian stiffness:  X,Y,Z in [N/m],  A,B,C in [Nm/rad]
    private double[] K = { 1000.0, 1000.0, 30.0, 300.0, 300.0, 300.0 };
    // Per-axis damping ratio [0.1 .. 1.0]
    private double[] D = { 0.7, 0.7, 0.7, 0.7, 0.7, 0.7 };
    private double ns_stiffness = 30.0;
    private double ns_damping = 0.7;

    // KUKA Sunrise valid ranges; every value is clamped to these (trans <= 5000 N/m, rot <= 300 Nm/rad)
    private static final double[] K_MAX = { 5000.0, 5000.0, 5000.0, 300.0, 300.0, 300.0 };

    // UI Options for Compliance. Each profile is a full per-axis {X,Y,Z,A,B,C} stiffness diagonal,
    // so the cabinet can enforce axis-aligned virtual fixtures in hardware (e.g. a flat table =
    // free X/Y + stiff Z).
    private String[] stiffness_profiles_ = {
        "Soft Z (Apple Pluck)",
        "Uniform Medium (Apple Pluck)",
        "Very Soft Z",
        "Flat table (free X/Y, stiff Z)",
        "Vertical rail (stiff X/Y, free Z)",
        "Stiff (firm walls)"
    };
    private double[][] stiffness_vals_ = {
        { 1000.0, 1000.0,   30.0, 300.0, 300.0, 300.0 }, // apple pluck: soft in Z
        {  100.0,  100.0,  100.0, 300.0, 300.0, 300.0 }, // apple pluck: uniform medium (preferred)
        {  800.0,  800.0,   10.0, 200.0, 200.0, 200.0 }, // apple pluck: extra-soft Z
        {   80.0,   80.0, 4000.0, 300.0, 300.0, 300.0 }, // plane fixture: free X/Y, hard Z wall
        { 4000.0, 4000.0,   80.0, 300.0, 300.0, 300.0 }, // rail fixture: free Z, walls in X/Y
        { 3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0 }  // firm everywhere
    };
    private String[] damping_options_ = { "0.3 (Underdamped)", "0.7 (Standard)", "1.0 (Critically Damped)" };
    private double[] damping_vals_ = { 0.3, 0.7, 1.0 };

    /**
     * Prompts the user on the SmartPAD to configure the connection.
     */
    public void request_user_config() {
        // Ask for Send Period
        int selectedButtonIndex = applicationUi.displayModalDialog(
                ApplicationDialogType.QUESTION,
                "Select the desired FRI send period [ms]:",
                send_periods_);
        send_period_ = Integer.valueOf(send_periods_[selectedButtonIndex]);
        getLogger().info("Send period set to: " + send_period_);

        // Ask for Remote IP
        selectedButtonIndex = applicationUi.displayModalDialog(
                ApplicationDialogType.QUESTION,
                "Select your remote IP address:",
                client_names_);
        client_name_ = client_names_[selectedButtonIndex];
        getLogger().info("Remote address set to: " + client_name_);

        // Ask for Stiffness Profile
        selectedButtonIndex = applicationUi.displayModalDialog(
                ApplicationDialogType.QUESTION,
                "Select Cartesian Stiffness (K diagonal):",
                stiffness_profiles_);
        K = stiffness_vals_[selectedButtonIndex];
        getLogger().info("Stiffness Profile set to: " + stiffness_profiles_[selectedButtonIndex]);

        // Ask for Damping Ratio
        selectedButtonIndex = applicationUi.displayModalDialog(
                ApplicationDialogType.QUESTION,
                "Select Damping Ratio (D0):",
                damping_options_);
        double d0 = damping_vals_[selectedButtonIndex];
        for (int i = 0; i < 6; i++) {
            D[i] = d0;
        }
        ns_damping = d0;
        getLogger().info("Damping Ratio set to: " + d0);

        // Setup the Cartesian Impedance Control Mode with per-axis stiffness & damping
        control_mode_ = new CartesianImpedanceControlMode();
        applyCartesianImpedance(control_mode_, K, D);

        getLogger().info("Control mode set to: Cartesian Impedance Control");
        getLogger().info("Stiffness (X, Y, Z, A, B, C): " + K[0] + ", " + K[1] + ", " + K[2] + 
                         ", " + K[3] + ", " + K[4] + ", " + K[5]);
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
        mode.setNullSpaceDamping(Math.max(0.1, Math.min(ns_damping, 1.0)));
    }

    /**
     * Initializes the Fast Robot Interface connection.
     */
    public void configure_fri() {
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
            return;
        }

        getLogger().info("FRI connection established.");
    }

    @Override
    public void initialize() {
        // Attach the end-effector tool so the controller accounts for its load in gravity
        // compensation. With loadData=0 in the template this is equivalent to a bare flange (safe);
        // once the EE is mounted and its loadData is filled + re-synced, the same call compensates it.
        ee_tool_ = getApplicationData().createFromTemplate(EE_TOOL_TEMPLATE);
        ee_tool_.attachTo(lbr_.getFlange());
        getLogger().info("Attached tool template '" + EE_TOOL_TEMPLATE + "' to the flange.");

        request_user_config();
        configure_fri();
    }

    @Override
    public void run() {
        // Execute the motion holding command. The overlay lets ROS 2 update the target dynamically.
        lbr_.getFlange().move(
            positionHold(control_mode_, -1, TimeUnit.SECONDS)
            .addMotionOverlay(fri_overlay_)
        );
    }
    
    @Override
    public void dispose() {
        if (fri_session_ != null) {
            getLogger().info("Disposing FRI session.");
            fri_session_.close();
        }
        super.dispose();
    }
}