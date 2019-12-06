/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;


public class DriveVector extends Command {

  double kVecPidEpsilon = .5; //inches
  PidConstants straightConstants = new PidConstants(.01, 0, 0);
  Vector2 _goalVector;
  double _curTime;
  PidController _straightController;
  Vector2 _startVector;
  boolean _isFieldOriented;
  
  public DriveVector(Vector2 goalVector, boolean isFieldOriented) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(DrivetrainSubsystem.getInstance());
    _goalVector = goalVector;
    _isFieldOriented = isFieldOriented;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _straightController = new PidController(straightConstants);
    _curTime = Timer.getFPGATimestamp();
    _straightController.setSetpoint(0);
    if (!_isFieldOriented){
      _goalVector = DrivetrainSubsystem.getInstance().getKinematicPosition().add(_goalVector.rotateBy(DrivetrainSubsystem.getInstance().getGyroscope().getAngle()));
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double curDist = _goalVector.subtract(DrivetrainSubsystem.getInstance().getKinematicPosition()).length;
    double mag = _straightController.calculate(curDist, Timer.getFPGATimestamp() - _curTime);
    DrivetrainSubsystem.getInstance().holonomicDrive(_goalVector.subtract(DrivetrainSubsystem.getInstance().getKinematicPosition()).scale(-mag/curDist), 0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return _goalVector.subtract(DrivetrainSubsystem.getInstance().getKinematicPosition()).length <= kVecPidEpsilon;
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
