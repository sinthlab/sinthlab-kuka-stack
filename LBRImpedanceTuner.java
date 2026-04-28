package application;

import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 * Utility application to test and feel different Cartesian Impedance profiles
 * directly on the KUKA controller.
 */
public class LBRImpedanceTuner extends RoboticsAPIApplication {
    @Inject
    private LBR lbr_;

    @Override
    public void initialize() {
        // Initialization if needed
    }

    @Override
    public void run() {
        getLogger().info("Initializing Impedance Tuner...");

        // 1. Prompt user for Stiffness Profile
        String[] stiffnessProfiles = {
            "Soft (200 N/m)", 
            "Medium (1000 N/m)", 
            "Stiff (3000 N/m)", 
            "Stiff Z (3000) / Soft XY (200)"
        };
        int stiffIdx = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION, 
            "Select Cartesian Stiffness Profile (XY & Z):", 
            stiffnessProfiles
        );

        // 2. Prompt user for Damping Profile
        String[] dampingProfiles = {
            "Underdamped (0.3 - Bouncy/Fluid)", 
            "Critically Damped (0.7 - Smooth)", 
            "Overdamped (1.0 - Honey/Sluggish)"
        };
        int dampIdx = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION, 
            "Select Damping Profile:", 
            dampingProfiles
        );

        // 3. Configure the Impedance Control Mode
        CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();

        // Apply selected stiffness
        switch (stiffIdx) {
            case 0: // Soft
                cartImp.parametrize(CartDOF.X).setStiffness(200.0);
                cartImp.parametrize(CartDOF.Y).setStiffness(200.0);
                cartImp.parametrize(CartDOF.Z).setStiffness(200.0);
                cartImp.parametrize(CartDOF.A).setStiffness(30.0);
                cartImp.parametrize(CartDOF.B).setStiffness(30.0);
                cartImp.parametrize(CartDOF.C).setStiffness(30.0);
                break;
            case 1: // Medium
                cartImp.parametrize(CartDOF.X).setStiffness(1000.0);
                cartImp.parametrize(CartDOF.Y).setStiffness(1000.0);
                cartImp.parametrize(CartDOF.Z).setStiffness(1000.0);
                cartImp.parametrize(CartDOF.A).setStiffness(100.0);
                cartImp.parametrize(CartDOF.B).setStiffness(100.0);
                cartImp.parametrize(CartDOF.C).setStiffness(100.0);
                break;
            case 2: // Stiff
                cartImp.parametrize(CartDOF.X).setStiffness(3000.0);
                cartImp.parametrize(CartDOF.Y).setStiffness(3000.0);
                cartImp.parametrize(CartDOF.Z).setStiffness(3000.0);
                // Keep rotational stiffness high so orientation doesn't droop
                cartImp.parametrize(CartDOF.A).setStiffness(300.0);
                cartImp.parametrize(CartDOF.B).setStiffness(300.0);
                cartImp.parametrize(CartDOF.C).setStiffness(300.0);
                break;
            case 3: // Stiff Z / Soft XY (Prevents Droop, allows planar pushing)
                cartImp.parametrize(CartDOF.Z).setStiffness(3000.0);
                cartImp.parametrize(CartDOF.X).setStiffness(200.0);
                cartImp.parametrize(CartDOF.Y).setStiffness(200.0);
                
                // Keep rotations stiff to keep end-effector level
                cartImp.parametrize(CartDOF.A).setStiffness(300.0);
                cartImp.parametrize(CartDOF.B).setStiffness(300.0);
                cartImp.parametrize(CartDOF.C).setStiffness(300.0);
                break;
        }

        // Apply selected damping
        double damping = 0.7;
        if (dampIdx == 0) damping = 0.3;
        else if (dampIdx == 1) damping = 0.7;
        else if (dampIdx == 2) damping = 1.0;

        cartImp.parametrize(CartDOF.ALL).setDamping(damping);

        // 3.5 Apply Null-Space (Elbow) Stiffness and Damping
        // Since the LBR has 7 DOFs, Cartesian control leaves 1 DOF redundant (the elbow).
        // Without null-space stiffness, the elbow might droop or drift wildly.
        double nsStiff = 50.0; // Default medium null-space stiffness
        if (stiffIdx == 0) nsStiff = 10.0;      // Soft elbow
        else if (stiffIdx == 1) nsStiff = 50.0; // Medium elbow
        else nsStiff = 100.0;                   // Stiff elbow (Max usually ~100)
        
        cartImp.setNullSpaceStiffness(nsStiff);
        cartImp.setNullSpaceDamping(damping);

        // 4. Activate the Hold
        getLogger().info("Applying PositionHold with selected Impedance settings.");
        getLogger().info("Go ahead and push the arm around to test the feel!");
        getLogger().info("Stop the application from the SmartPad when done.");

        // -1 duration means to hold infinitely until user stops the app
        PositionHold posHold = new PositionHold(cartImp, -1, null);
        lbr_.move(posHold);
    }
}
