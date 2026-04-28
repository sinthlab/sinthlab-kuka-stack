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
            "Soft Z (Apple Pluck) [1000,1000,30]", 
            "Very Soft Z [800,800,10]", 
            "Medium Cartesian [100,100,100]", 
            "Stiff Cartesian [1000,1000,1000]",
            "Anti-Droop (Soft XY, Stiff Z) [300,300,4000]"
        };
        int stiffIdx = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION, 
            "Select Cartesian Stiffness Profile (K diagonal):", 
            stiffnessProfiles
        );

        // 2. Prompt user for Damping Profile
        String[] dampingProfiles = { 
            "0.3 (Underdamped)", 
            "0.7 (Standard)", 
            "1.0 (Critically Damped)" 
        };
        int dampIdx = getApplicationUI().displayModalDialog(
            ApplicationDialogType.QUESTION, 
            "Select Damping Ratio (D0):", 
            dampingProfiles
        );

        // 3. Configure the Impedance Control Mode
        CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();

        // Exact arrays from LbrImpedanceControlServer.java
        double[][] stiffnessVals = {
            { 1000.0, 1000.0, 30.0, 300.0, 300.0, 300.0 }, // Soft Z
            { 800.0, 800.0, 10.0, 200.0, 200.0, 200.0 },   // Very Soft Z
            { 100.0, 100.0, 100.0, 300.0, 300.0, 300.0 },  // Medium
            { 1000.0, 1000.0, 1000.0, 300.0, 300.0, 300.0 },// Stiff
            { 300.0, 300.0, 4000.0, 300.0, 300.0, 300.0 }  // Anti-Droop (Soft XY, Stiff Z)
        };
        
        double[] K = stiffnessVals[stiffIdx];
        
        // Apply strictly to Cartesian Translation & Rotation
        cartImp.parametrize(CartDOF.X).setStiffness(K[0]);
        cartImp.parametrize(CartDOF.Y).setStiffness(K[1]);
        cartImp.parametrize(CartDOF.Z).setStiffness(K[2]);
        cartImp.parametrize(CartDOF.A).setStiffness(K[3]);
        cartImp.parametrize(CartDOF.B).setStiffness(K[4]);
        cartImp.parametrize(CartDOF.C).setStiffness(K[5]);

        // Apply selected damping
        double[] dampingVals = { 0.3, 0.7, 1.0 };
        double damping = dampingVals[dampIdx];

        cartImp.parametrize(CartDOF.ALL).setDamping(damping);

        // 3.5 Apply Null-Space (Elbow) Stiffness and Damping
        // As defined in LbrImpedanceControlServer.java
        double nsStiff = 30.0;
        
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
