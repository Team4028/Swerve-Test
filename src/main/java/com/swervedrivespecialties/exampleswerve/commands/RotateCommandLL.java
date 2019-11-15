/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RotateCommandLL extends Command {

  DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();
  private double _theta;
  private double _rotateCommand;
  private boolean _run;

  private NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");
	private NetworkTableEntry tx = nt.getEntry("tx");

  public RotateCommandLL(boolean run) {
    requires(_drive);
    //_theta = theta;
    _run = run;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    _theta = tx.getDouble(0);

    if(Math.abs(_theta) > 0.5){
    _rotateCommand = -Math.copySign(Math.pow(Math.copySign(0.11, _theta) + (0.01 * _theta), 2), _theta);
    }
    else{
      _rotateCommand = 0;
    }
    _drive.holonomicDrive(new Vector2(0,0), _rotateCommand, _drive.getFieldOriented());
    SmartDashboard.putNumber("LL Value", _theta);
    SmartDashboard.putNumber("Rotate Command", _rotateCommand);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return _run;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
  public double get_rotateCommand() {
    return _rotateCommand;
  }
}
