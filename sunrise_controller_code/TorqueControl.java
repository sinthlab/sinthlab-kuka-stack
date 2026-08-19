/*
 * TorqueControl - Sunrise FRI application for the TORQUE experiments (restricted-plane + maze).
 * Source: idra-lab/kuka_lbr_control (assets/TorqueControl.java), Apache-2.0.
 *
 * Opens the FRI session as a JOINT overlay in ClientCommandMode.TORQUE with a zero-stiffness
 * JointImpedanceControlMode -> the CABINET compensates gravity/friction/Coriolis and the ROS
 * impedance controllers (cartesian_impedance_controller / joint_impedance_controller) add joint
 * torques on top. This is what makes the arm compliant WITHOUT free-fall.
 *
 * Pair with: config/lbr_system_config_torque.yaml (client_command_mode: torque).
 * The apple-pluck / perturb experiments keep using LbrImpedanceControlServer.java (position mode).
 *
 * >>> SETUP (matches the other servers in this folder):
 *   - Remote IP is chosen at runtime via a SmartPad dialog (client_names_ list); add your PC's IP there
 *     if it isn't already listed.
 *   - EE_TOOL_TEMPLATE = "SinthLabIiwa7EE" must match the tool template in your Sunrise project (the one
 *     you ran the load-data Determine on).
 */
package lbr_fri_ros2;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.Arrays;

import javax.inject.Inject;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.IApplicationUI;
import com.kuka.roboticsAPI.motionModel.controlModeModel.*;
import com.kuka.connectivity.fastRobotInterface.*;

public class TorqueControl extends RoboticsAPIApplication {
	// members
	@Inject
	private LBR lbr_;
	@Inject
	private IApplicationUI applicationUi;
	private Controller lbr_controller_;

	// End-effector tool TEMPLATE name — MUST match the tool template in your Sunrise project
	// (RoboticsAPI.data.xml). Attaching it lets the cabinet gravity-compensate the payload; its
	// loadData must match what is physically mounted (run the SmartPad "Determine").
	private static final String EE_TOOL_TEMPLATE = "SinthLabIiwa7EE";
	private Tool ee_tool_;

	// control mode
	private enum CONTROL_MODE {
		POSITION_CONTROL,
		JOINT_IMPEDANCE_CONTROL,
		CARTESIAN_IMPEDANCE_CONTROL;
	}

	// convert enum to string array, see https://stackoverflow.com/questions/13783295/getting-all-names-in-an-enum-as-a-string
	public static String[] getNames(Class<? extends Enum<?>> e) {
	    return Arrays.toString(e.getEnumConstants()).replaceAll("^.|.$", "").split(", ");
	}

	// FRI parameters
	private String client_name_;
	private String[] client_names_ = { "172.31.1.148", "192.170.10.100" };
	// FRI send period [ms], chosen on the SmartPad. It MUST match controller_manager update_rate in
	// config/torque_controllers.yaml: update_rate_hz = 1000 / send_period_ms (so 5 -> 200, 10 -> 100,
	// 1 -> 1000). Pick 5 to match the shipped config. 1 ms / 1000 Hz needs a real-time host; if the FRI
	// session hangs in "Monitoring (Wait)" (connection quality never GOOD), pick a larger value.
	private int send_period_;
	private String[] send_periods_ = { "1", "2", "5", "10" };

	private FRIConfiguration fri_configuration_;
	private FRISession fri_session_;
	private FRIJointOverlay fri_overlay_;

	private AbstractMotionControlMode control_mode_;
	private String[] control_modes_ = getNames(CONTROL_MODE.class);
	private ClientCommandMode command_mode_;
	private String[] command_modes_ = getNames(ClientCommandMode.class);
	// methods
	public void request_user_config() {
		// Ask for Send Period (SmartPad dialog, like LbrImpedanceControlServer). Keep consistent with
		// update_rate in config/torque_controllers.yaml: update_rate_hz = 1000 / send_period_ms.
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

		control_mode_= 	new JointImpedanceControlMode(0.0,0.0,0.0,0.0,0.0,0.0,0.0);
		command_mode_ = ClientCommandMode.TORQUE;
		getLogger().info("Client command mode set to: " + command_mode_.name());
	}

	public void configure_fri() {
		fri_configuration_ = FRIConfiguration.createRemoteConfiguration(lbr_, client_name_);
		fri_configuration_.setSendPeriodMilliSec(send_period_);

        getLogger().info("Creating FRI connection to " + fri_configuration_.getHostName());
        getLogger().info(
			"SendPeriod: " + fri_configuration_.getSendPeriodMilliSec() + "ms |"
	        + " ReceiveMultiplier: " + fri_configuration_.getReceiveMultiplier()
        );

        fri_session_ = new FRISession(fri_configuration_);
        fri_overlay_ = new FRIJointOverlay(fri_session_, command_mode_);

        fri_session_.addFRISessionListener(new IFRISessionListener() {
	    	@Override
	    	public void onFRISessionStateChanged(FRIChannelInformation friChannelInformation) {
	    		getLogger().info("Session State change " + friChannelInformation.getFRISessionState().toString() );
	    	}

	    	@Override
	    	public void onFRIConnectionQualityChanged(FRIChannelInformation friChannelInformation) {
		    	getLogger().info("Quality change signalled "+friChannelInformation.getQuality());
		    	getLogger().info("Jitter "+friChannelInformation.getJitter());
		    	getLogger().info("Latency "+friChannelInformation.getLatency());
	    	}
    	});

        // try to connect
        try {
        	fri_session_.await(60, TimeUnit.SECONDS);
        } catch (final TimeoutException e) {
        	getLogger().error(e.getLocalizedMessage());
        	getLogger().error("Connection timeout: Current Timeout limit = 60 sec");
        	return;
        }

        getLogger().info("FRI connection established.");
	}

	@Override
	public void initialize() {
		ObjectFrame lbr_flange = lbr_.getFlange();
		ee_tool_ = getApplicationData().createFromTemplate(EE_TOOL_TEMPLATE);
		ee_tool_.attachTo(lbr_flange);
		getLogger().info("End Effector position:" + lbr_flange.getX()/1000.0 + " " + lbr_flange.getY()/1000.0 + " " + lbr_flange.getZ()/1000.0);

        lbr_controller_ = (Controller) getContext().getControllers().toArray()[0];
        lbr_ = (LBR) lbr_controller_.getDevices().toArray()[0];

        // set FRI parameters
		request_user_config();

		// configure the FRI
		configure_fri();
	}

	@Override
	public void run() {
		// run the FRI
		lbr_.move(positionHold(control_mode_, -1, null).addMotionOverlay(fri_overlay_));
		return;
	}

	@Override
	public void dispose() {
		// close connection
		getLogger().info("Disposing FRI session.");
		fri_session_.close();

		super.dispose();
	}

	/**
	 * main
	 *
	 * @param args
	 */
	public static void main(final String[] args) {
		TorqueControl app = new TorqueControl();
		app.runApplication();
	}
}
