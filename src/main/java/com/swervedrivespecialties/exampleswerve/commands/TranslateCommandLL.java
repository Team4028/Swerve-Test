/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TranslateCommandLL extends Command {

  private NetworkTable nt = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry camtran = nt.getEntry("camtran");
  
  private double xtran;
  private double forward;
  private double strafe;
  private double gyroYaw;
  private double llYaw;
  private double alpha;
  private double p = 0.01;
  private double ff = 0.07;
  private double[] defaultValArray = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  private DrivetrainSubsystem _driveTrainSubsystem = DrivetrainSubsystem.getInstance();

  public TranslateCommandLL() {
    requires(_driveTrainSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //xtran = camtran.getDoubleArray(defaultValArray)[0];
    gyroYaw = _driveTrainSubsystem.getGyroscope().getAngle().toDegrees();
    llYaw = camtran.getDoubleArray(defaultValArray)[3];
    alpha = llYaw + gyroYaw;
    SmartDashboard.putNumber("alpha angle value", alpha);

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
