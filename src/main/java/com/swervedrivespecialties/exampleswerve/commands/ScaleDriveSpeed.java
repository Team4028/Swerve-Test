/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;

public class ScaleDriveSpeed extends Command {
  double slow=0.5;
  double med=0.6;
  double fast=0.8;
  Button _a, _b, _y;
  DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();
  public ScaleDriveSpeed(Button a, Button b, Button y) 
  {
    _a=a;
    _b=b;
    _y=y;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
    if(_a.get())
    {
      _drive.setSpeedScaling(slow);
    }
    else if(_b.get())
    {
      _drive.setSpeedScaling(med);
    }
    else if(_y.get())
    {
      _drive.setSpeedScaling(fast);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
}
