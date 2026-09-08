package lbr_fri_ros2;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicLong;

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
// Generated per-project from the Sunrise I/O configuration. If this import does not resolve,
// the media flange is not in your project's I/O configuration yet -- add it in Sunrise Workbench
// (Station Setup -> I/O Configuration) and the class will be generated. See the cue-server
// section below.
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;

/**
 * LbrImpedanceControlServer
 * 
 * Hardware-native Cartesian Impedance Control application for the KUKA Sunrise cabinet.
 * This runs pure Cartesian Impedance locally at 1000Hz while listening to joint
 * position targets via FRI from the ROS 2 driver.
 *
 * It ALSO hosts a small TCP "cue server" (see the CUE SERVER section below) that lets a ROS 2
 * orchestrator pulse a media-flange digital output. That line drives the end effector's NeoPixel
 * cue ring through an optocoupler. FRI carries joint commands only and cannot carry an arbitrary
 * application call, which is why the cue rides its own socket rather than the FRI channel.
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
        "Rail guide (uniform 1000)",
        "Stiff (firm walls)",
        "Maze compliant (uniform 400)",
        "Maze walls (X lock, Y/Z firm)",
        "Maze walls + easy guiding (rot 120)"
    };
    private double[][] stiffness_vals_ = {
        { 1000.0, 1000.0,   30.0, 300.0, 300.0, 300.0 }, // apple pluck: soft in Z
        {  100.0,  100.0,  100.0, 300.0, 300.0, 300.0 }, // apple pluck: uniform medium (preferred)
        {  800.0,  800.0,   10.0, 200.0, 200.0, 200.0 }, // apple pluck: extra-soft Z
        {   80.0,   80.0, 4000.0, 300.0, 300.0, 300.0 }, // plane fixture: free X/Y, hard Z wall
        { 1000.0, 1000.0, 1000.0, 300.0, 300.0, 300.0 }, // fixture experiments: uniform guide
        { 3000.0, 3000.0, 3000.0, 300.0, 300.0, 300.0 }, // firm everywhere
        { 400.0, 400.0, 400.0, 300.0, 300.0, 300.0 },    // maze: compliant uniform guide
        // MAZE (vertical Y-Z plane, restricted_axis "x"): ANISOTROPIC and the one to use.
        //   X 2500 = lock the radial axis -> the plane is enforced by the CABINET, hard.
        //   Y/Z 1000 = firm corridor walls (~10 N per cm of penetration).
        // The corridor INTERIOR still feels free: there the fixture's projection returns the measured
        // pose, so spring error ~ 0 and force ~ 0 regardless of these numbers. Only the walls and the
        // locked axis see a real error. This is why a uniform profile could never win -- 400 made the
        // walls mushy, 3000 made everything heavy.
        { 2500.0, 1000.0, 1000.0, 300.0, 300.0, 300.0 }, // maze: X lock + firm Y/Z walls
        // MAZE, tuned for SMOOTH guiding. IDENTICAL translational stiffness to the profile above --
        // the maze's constraints are ALL translational (X = the plane lock, Y/Z = the corridor walls),
        // so those must stay firm or the fixture itself goes mushy. ONLY the rotational terms are
        // relaxed:
        //   rot 300 -> 120. Orientation locking defines nothing about the maze; it merely stops the
        //   tool twisting. But holding orientation while translating is the EXPENSIVE motion (it is
        //   what made the old maze start feel like treacle), so this is the one knob that reduces drag
        //   on guiding WITHOUT weakening the plane or the walls.
        // Trade-off: the tool may twist a little more -- watch the apple angle.
        { 2500.0, 1000.0, 1000.0, 120.0, 120.0, 120.0 }  // maze: firm constraints, easy guiding
    };
    private String[] damping_options_ = { "0.3 (Underdamped)", "0.7 (Standard)", "1.0 (Critically Damped)" };
    private double[] damping_vals_ = { 0.3, 0.7, 1.0 };

    // =======================================================================================
    // CUE SERVER -- media-flange digital output, driven from ROS 2 over a TCP socket
    // =======================================================================================
    // WHY A SOCKET AND NOT FRI: FRI carries joint commands and robot state; it has no channel for
    // "run a cue now" unless boolean FRI I/O is declared in the Sunrise project AND the ROS side
    // gains a matching command interface (that part lives in lbr_ros2_control, which we do not
    // own). A socket keeps the whole feature inside code we control.
    //
    // TIMING: the ack below carries a cabinet-side timestamp, so the ROS orchestrator can bracket
    // the cue between its own send time and the ack. That bounds the ROS->cabinet leg, which is the
    // ONLY leg where this approach differs from FRI I/O -- everything downstream (cabinet I/O
    // cycle, optocoupler, the board's debounce, the LED refresh) is common to both and is what
    // actually dominates. Calibrate that fixed offset once with a scope; do not assume it.
    //
    // SAFETY: this whole subsystem is isolated from the motion application. Every socket and I/O
    // operation is caught, the threads are daemons, and a failure here logs and retries -- it must
    // never be able to disturb positionHold() or the FRI session.
    private static final int CUE_PORT = 30300;              // FRI uses 30200; keep these distinct
    private static final int CUE_DEFAULT_PULSE_MS = 50;     // comfortably clears the board's 5 ms
                                                            // debounce; the board owns cue LENGTH
                                                            // in "pulse" mode
    private static final int CUE_MAX_PULSE_MS = 10000;      // ceiling for "follow" mode, where the
                                                            // pulse width IS the cue length
    private static final int CUE_TICK_MS = 1;               // deassert-timer resolution

    @Inject
    private MediaFlangeIOGroup media_flange_;

    private volatile boolean cue_running_ = false;
    private ServerSocket cue_server_socket_;
    private Thread cue_accept_thread_;
    private Thread cue_pulse_thread_;
    // Deassert deadline from System.nanoTime(); 0 means "output is not asserted".
    private final AtomicLong cue_deassert_ns_ = new AtomicLong(0L);
    private final AtomicLong cue_seq_ = new AtomicLong(0L);

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

    // -----------------------------------------------------------------------------------
    // ==> THE ONE PLACE THAT TOUCHES THE MEDIA FLANGE. ADAPT THIS TO YOUR FLANGE VARIANT. <==
    // -----------------------------------------------------------------------------------
    // The generated MediaFlangeIOGroup's setter name depends on which media flange the robot has
    // and how the I/O was named in the Sunrise project's I/O configuration. Common variants:
    //
    //     media_flange_.setOutputX3Pin1(on);   // Media Flange IO / electrical  <-- assumed here
    //     media_flange_.setOutputX3Pin2(on);
    //     media_flange_.setLEDBlue(on);        // Media Flange Touch
    //
    // In Sunrise Workbench, type "media_flange_." and let autocomplete list what your project
    // actually generated, then keep the one that maps to the pin you wired the optocoupler to.
    // Nothing else in this file needs to change.
    private void setCueOutput(boolean on) {
        try {
            media_flange_.setOutputX3Pin1(on);
        } catch (Exception e) {
            // An I/O fault must not propagate into the motion application.
            getLogger().error("Cue output write failed: " + e.toString());
        }
    }

    /**
     * Asserts the cue line for pulse_ms and schedules its release.
     *
     * The deadline is published BEFORE the output is asserted so the timer thread can never
     * observe an asserted output with no deadline (which would latch the cue on). Overlapping
     * requests simply extend the deadline rather than racing.
     */
    private void assertCue(int pulse_ms) {
        int ms = Math.max(1, Math.min(pulse_ms, CUE_MAX_PULSE_MS));
        long deadline = System.nanoTime() + (long) ms * 1000000L;
        if (deadline == 0L) {
            deadline = 1L;      // 0 is the sentinel for "not asserted"; nanoTime() may be negative
        }
        cue_deassert_ns_.set(deadline);
        setCueOutput(true);
    }

    /** Releases the cue line immediately. */
    private void releaseCue() {
        cue_deassert_ns_.set(0L);
        setCueOutput(false);
    }

    /**
     * Handles one command line. Protocol is line-oriented ASCII, deliberately trivial:
     *
     *   CUE [ms]   assert the line for [ms] (default CUE_DEFAULT_PULSE_MS), then release
     *   OFF        release the line now
     *   PING       liveness check
     *   STATUS     report the line state
     *
     * Every reply ends with a sequence number and a cabinet timestamp:
     *
     *   OK <seq> <nanoTime_ns> <wallClock_ms>
     *
     * <seq> lets the ROS side detect a dropped or duplicated cue -- the failure mode that
     * silently corrupts behavioural data instead of announcing itself. <nanoTime_ns> is monotonic
     * and is the one to use for intervals; <wallClock_ms> is only meaningful if the cabinet's
     * clock is synchronised, which it generally is not.
     */
    private String handleCueCommand(String line) {
        String cmd = line.trim();
        if (cmd.length() == 0) {
            return null;                    // ignore blank lines (keep-alives)
        }
        String upper = cmd.toUpperCase();
        long seq = cue_seq_.incrementAndGet();
        String stamp = " " + seq + " " + System.nanoTime() + " " + System.currentTimeMillis();

        if (upper.equals("PING")) {
            return "PONG" + stamp;
        }
        if (upper.equals("OFF")) {
            releaseCue();
            return "OK" + stamp;
        }
        if (upper.equals("STATUS")) {
            return "OK" + stamp + " asserted=" + (cue_deassert_ns_.get() != 0L)
                   + " default_ms=" + CUE_DEFAULT_PULSE_MS;
        }
        if (upper.equals("CUE") || upper.startsWith("CUE ")) {
            int ms = CUE_DEFAULT_PULSE_MS;
            if (upper.length() > 4) {
                try {
                    ms = Integer.parseInt(cmd.substring(4).trim());
                } catch (NumberFormatException e) {
                    return "ERR" + stamp + " bad pulse length";
                }
            }
            assertCue(ms);
            return "OK" + stamp + " pulse_ms=" + Math.max(1, Math.min(ms, CUE_MAX_PULSE_MS));
        }
        return "ERR" + stamp + " unknown command";
    }

    /**
     * Serves one connected client until it disconnects. One client at a time is served -- the
     * experiment orchestrator. Keep the connection OPEN across trials: a fresh TCP handshake per
     * cue would add a round trip to the very latency this design is trying to keep small.
     */
    private void serveCueClient(Socket socket) {
        BufferedReader in = null;
        Writer out = null;
        try {
            socket.setTcpNoDelay(true);     // ESSENTIAL: Nagle would buffer these tiny writes and
                                            // add tens of milliseconds to a timing-critical path
            socket.setKeepAlive(true);      // let the OS reap a peer that vanished without a FIN
            in = new BufferedReader(new InputStreamReader(socket.getInputStream(), "US-ASCII"));
            out = new OutputStreamWriter(socket.getOutputStream(), "US-ASCII");
            getLogger().info("Cue client connected from " + socket.getRemoteSocketAddress());

            String line;
            while (cue_running_ && (line = in.readLine()) != null) {
                String reply = handleCueCommand(line);
                if (reply != null) {
                    out.write(reply);
                    out.write("\n");
                    out.flush();
                }
            }
        } catch (SocketException e) {
            getLogger().info("Cue client disconnected: " + e.getMessage());
        } catch (IOException e) {
            getLogger().warn("Cue client I/O error: " + e.toString());
        } finally {
            // A client going away must never leave the ring latched on.
            releaseCue();
            closeQuietly(in);
            closeQuietly(out);
            closeQuietly(socket);
        }
    }

    private void closeQuietly(java.io.Closeable c) {
        if (c != null) {
            try {
                c.close();
            } catch (IOException e) {
                // nothing useful to do
            }
        }
    }

    private void closeQuietly(Socket s) {
        if (s != null) {
            try {
                s.close();
            } catch (IOException e) {
                // nothing useful to do
            }
        }
    }

    // ServerSocket only implements Closeable from Java 7 onward; its own overload keeps this
    // compiling on the older Sunrise toolchains.
    private void closeQuietly(ServerSocket s) {
        if (s != null) {
            try {
                s.close();
            } catch (IOException e) {
                // nothing useful to do
            }
        }
    }

    /**
     * Starts the cue server: an accept loop and a deassert timer, both daemon threads.
     *
     * Failure to start is logged and swallowed. The experiment must still run without a cue ring.
     */
    private void startCueServer() {
        try {
            cue_server_socket_ = new ServerSocket();
            cue_server_socket_.setReuseAddress(true);
            cue_server_socket_.bind(new InetSocketAddress(CUE_PORT));
        } catch (IOException e) {
            getLogger().error("Cue server could not bind port " + CUE_PORT + ": " + e.toString()
                              + " -- continuing WITHOUT the cue path.");
            cue_server_socket_ = null;
            return;
        }

        cue_running_ = true;
        setCueOutput(false);        // known state at startup

        cue_pulse_thread_ = new Thread(new Runnable() {
            @Override
            public void run() {
                while (cue_running_) {
                    try {
                        long deadline = cue_deassert_ns_.get();
                        if (deadline != 0L && System.nanoTime() >= deadline) {
                            // CAS so a cue that re-armed in the meantime is not cut short
                            if (cue_deassert_ns_.compareAndSet(deadline, 0L)) {
                                setCueOutput(false);
                            }
                        }
                        Thread.sleep(CUE_TICK_MS);
                    } catch (InterruptedException e) {
                        return;
                    } catch (Exception e) {
                        getLogger().error("Cue timer error: " + e.toString());
                    }
                }
            }
        }, "cue-pulse-timer");
        cue_pulse_thread_.setDaemon(true);
        cue_pulse_thread_.start();

        cue_accept_thread_ = new Thread(new Runnable() {
            @Override
            public void run() {
                while (cue_running_) {
                    try {
                        Socket socket = cue_server_socket_.accept();
                        serveCueClient(socket);
                    } catch (IOException e) {
                        if (cue_running_) {
                            getLogger().warn("Cue accept failed: " + e.toString());
                            try {
                                Thread.sleep(200);      // do not spin on a persistent fault
                            } catch (InterruptedException ie) {
                                return;
                            }
                        }
                    } catch (Exception e) {
                        getLogger().error("Cue server error: " + e.toString());
                    }
                }
            }
        }, "cue-accept");
        cue_accept_thread_.setDaemon(true);
        cue_accept_thread_.start();

        getLogger().info("Cue server listening on TCP " + CUE_PORT
                         + " (CUE [ms] | OFF | PING | STATUS), default pulse "
                         + CUE_DEFAULT_PULSE_MS + " ms.");
    }

    /** Stops the cue server and guarantees the output is left low. */
    private void stopCueServer() {
        cue_running_ = false;
        try {
            releaseCue();
        } catch (Exception e) {
            getLogger().error("Cue release on shutdown failed: " + e.toString());
        }
        closeQuietly(cue_server_socket_);
        if (cue_accept_thread_ != null) {
            cue_accept_thread_.interrupt();
        }
        if (cue_pulse_thread_ != null) {
            cue_pulse_thread_.interrupt();
        }
        getLogger().info("Cue server stopped.");
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

        // Started BEFORE the FRI handshake so the cue path is already answering while
        // configure_fri() blocks for up to 60 s waiting for the ROS 2 client.
        //
        // Belt and braces: startCueServer() already handles a failed bind, but the whole call is
        // caught as well. The stated rule for this subsystem is that it can never stop the
        // experiment running, and that has to hold for an unanticipated failure too.
        try {
            startCueServer();
        } catch (Exception e) {
            getLogger().error("Cue server failed to start: " + e.toString()
                              + " -- continuing WITHOUT the cue path.");
        }

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
        // Stop the cue path first, so the output is guaranteed low before anything else tears
        // down -- an application abort must never leave the ring latched on.
        stopCueServer();
        if (fri_session_ != null) {
            getLogger().info("Disposing FRI session.");
            fri_session_.close();
        }
        super.dispose();
    }
}