package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.util.BeakXboxController;

import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.common.math.Vector2;

public class DriveCommand extends Command 
{
    BeakXboxController _driverController;
    DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();
    double heccyscale=0.5;

    public DriveCommand(BeakXboxController driverController) 
    {
        _driverController=driverController;
        requires(DrivetrainSubsystem.getInstance());
    }
    @Override
    protected void initialize() 
    {
    }

    @Override
    protected void execute() 
    {
        if(_driverController.a.get())
        {
            _drive.setSpeedScaling(0.3);
        }
        else if(_driverController.b.get())
        {
           _drive.setSpeedScaling(0.6);

        }
        else if(_driverController.y.get())
        {
            _drive.setSpeedScaling(0.75);//hehe,idiot

        }

        double forward = -_driverController.getRawAxis(1);
        // Square the forward stick
        forward = Math.copySign(Math.pow(forward, 2.0), forward)*_drive.getSpeedScaling();

        double strafe = -_driverController.getRawAxis(0);
        // Square the strafe stick
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe)*_drive.getSpeedScaling();

        double rotation = -_driverController.getRawAxis(4);
        // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation)*_drive.getSpeedScaling();

        _drive.holonomicDrive(new Vector2(forward, strafe), rotation, _drive.getFieldOriented());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
