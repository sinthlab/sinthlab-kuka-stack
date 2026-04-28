package lbr_fri_ros2;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
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
    private double[] K = { 1000.0, 1000.0, 30.0, 300.0, 300.0, 300.0 };
    private double D0 = 0.7;
    private double ns_stiffness = 30.0;

    // UI Options for Compliance
    private String[] stiffness_profiles_ = {
        "Soft Z (Apple Pluck)", 
        "Very Soft Z", 
        "Medium Cartesian", 
        "Stiff Cartesian"
    };
    private double[][] stiffness_vals_ = {
        { 1000.0, 1000.0, 30.0, 300.0, 300.0, 300.0 }, // Soft Z
        { 800.0, 800.0, 10.0, 200.0, 200.0, 200.0 },   // Very Soft Z
        { 100.0, 100.0, 100.0, 300.0, 300.0, 300.0 },  // Medium
        { 1000.0, 1000.0, 1000.0, 300.0, 300.0, 300.0 }// Stiff
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
        D0 = damping_vals_[selectedButtonIndex];
        getLogger().info("Damping Ratio set to: " + D0);

        // Setup the Cartesian Impedance Control Mode
        control_mode_ = new CartesianImpedanceControlMode();
        
        // Apply strictly to Cartesian Translation
        control_mode_.parametrize(CartDOF.X).setStiffness(K[0]);
        control_mode_.parametrize(CartDOF.Y).setStiffness(K[1]);
        control_mode_.parametrize(CartDOF.Z).setStiffness(K[2]);
        
        // Apply strictly to Cartesian Rotation
        control_mode_.parametrize(CartDOF.A).setStiffness(K[3]);
        control_mode_.parametrize(CartDOF.B).setStiffness(K[4]);
        control_mode_.parametrize(CartDOF.C).setStiffness(K[5]);

        // Standard 0.7 Damping Ratio for smooth recovery
        control_mode_.parametrize(CartDOF.X).setDamping(D0);
        control_mode_.parametrize(CartDOF.Y).setDamping(D0);
        control_mode_.parametrize(CartDOF.Z).setDamping(D0);
        control_mode_.parametrize(CartDOF.A).setDamping(D0);
        control_mode_.parametrize(CartDOF.B).setDamping(D0);
        control_mode_.parametrize(CartDOF.C).setDamping(D0);

        // Nullspace compliance
        control_mode_.setNullSpaceStiffness(ns_stiffness);
        control_mode_.setNullSpaceDamping(D0);

        getLogger().info("Control mode set to: Cartesian Impedance Control");
        getLogger().info("Stiffness (X, Y, Z, A, B, C): " + K[0] + ", " + K[1] + ", " + K[2] + 
                         ", " + K[3] + ", " + K[4] + ", " + K[5]);
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