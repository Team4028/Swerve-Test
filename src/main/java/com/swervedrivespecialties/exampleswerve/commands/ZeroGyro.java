/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.drivers.Gyroscope;
import edu.wpi.first.wpilibj.command.Command;

public class ZeroGyro extends Command 
{
  Gyroscope _gyro = DrivetrainSubsystem.getInstance().getGyroscope();
  
  public ZeroGyro() 
  {
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
    _gyro.setAdjustmentAngle(_gyro.getUnadjustedAngle());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() 
  {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() 
  {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() 
  {
  }
}
