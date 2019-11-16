package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.util.BeakXboxController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.common.math.Vector2;

public class DriveCommand extends Command 
{
    BeakXboxController _driverController;
    DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();
    double heccyscale=0.5;
    private boolean useLL = false;
    private double _theta;
    private double forward, strafe, rotation;

    private NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");
	private NetworkTableEntry tx = nt.getEntry("tx");

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
        _theta = tx.getDouble(0);
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
            _drive.setSpeedScaling(1);//hehe
        }
        if(_driverController.x.get()){
            useLL = !useLL;
        }

        forward = -_driverController.getRawAxis(1);
        // Square the forward stick
        forward = Math.copySign(Math.pow(forward, 2.0), forward)*_drive.getSpeedScaling();

        strafe = -_driverController.getRawAxis(0);
        // Square the strafe stick
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe)*_drive.getSpeedScaling();
        /*if(useLL){
            if(Math.abs(_theta) > 0.5){
                rotation = -Math.copySign(Math.pow(Math.copySign(0.11, _theta) + (0.01 * _theta), 2), _theta);
                }
            else{
                  rotation = 0;
                }
        }
        else{*/
        rotation = -_driverController.getRawAxis(4);
        //}
        //Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation)*_drive.getSpeedScaling();


        _drive.holonomicDrive(new Vector2(forward, strafe), rotation, _drive.getFieldOriented());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
