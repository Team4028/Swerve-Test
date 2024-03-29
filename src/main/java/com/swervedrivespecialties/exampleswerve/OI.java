package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.commands.DriveCommand;
import com.swervedrivespecialties.exampleswerve.commands.ScaleDriveSpeed;
import com.swervedrivespecialties.exampleswerve.commands.ToggleFieldOriented;
import com.swervedrivespecialties.exampleswerve.commands.ZeroGyro;
import com.swervedrivespecialties.exampleswerve.util.BeakXboxController;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;

/**
 * This class interfaces with the Driver/Operator Station Lead Student:
 */
public class OI {
	private BeakXboxController _driverController;
	private BeakXboxController _operatorController;
	private BeakXboxController _engineerController;

	// =====================================================================================
	// Define Singleton Pattern
	// =====================================================================================
	private static OI _instance = new OI();

	public static OI getInstance() {
		return _instance;
	}

	// private constructor for singleton pattern
	private OI() 	
	{	
		// =========== Driver ======================================
		_driverController = new BeakXboxController(0);
		//==========================================================

		// Driver Controller -> Command Mapping
		_driverController.leftStick.whileActive(new DriveCommand(_driverController));	
		_driverController.leftStick.whenReleased(new DriveCommand(_driverController));
		_driverController.rightStick.whileActive(new DriveCommand(_driverController));	
        _driverController.rightStick.whenReleased(new DriveCommand(_driverController));
        
        _driverController.back.whenPressed(new ZeroGyro());

        _driverController.start.whenPressed(new ToggleFieldOriented());



    }

	private Command ScaleDriveSpeed(Button a, Button b, Button y) {
		return null;
	}
}
// package com.swervedrivespecialties.exampleswerve;

// import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.buttons.JoystickButton;
// import edu.wpi.first.wpilibj.command.InstantCommand;

// public class OI {
//     /*
//        Add your joysticks and buttons here
//      */
//     private Joystick primaryJoystick = new Joystick(0);

//     public OI() {
//         // Back button zeroes the drivetrain
//         new JoystickButton(primaryJoystick, 7).whenPressed(
//                 new InstantCommand(() -> DrivetrainSubsystem.getInstance().getGyroscope().setAdjustmentAngle(
//                         DrivetrainSubsystem.getInstance().getGyroscope().getUnadjustedAngle())
//                 )
//         );

//     }

//     public Joystick getPrimaryJoystick() {
//         return primaryJoystick;
//     }
// }
