/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.command.Command;

public class RotateDrivetrain extends Command {
  private DrivetrainSubsystem _drive = DrivetrainSubsystem.getInstance();
  private double p = 0.05;
  private double rotateCommand;
  public double _angle;
  public RotateDrivetrain(double angle) {
    _angle = angle;
  }


  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Math.abs(_angle) > 0.5){
      rotateCommand = p*_angle;
    }
    else{
      rotateCommand = 0;
    }
    _drive.holonomicDrive(new Vector2(0, 0), rotateCommand, _drive.getFieldOriented());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
