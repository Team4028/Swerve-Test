/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.autonomous.TestTrajectories;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PathWithLL extends CommandGroup {

  private DrivetrainSubsystem _driveTrainSubsystem = DrivetrainSubsystem.getInstance();
  private TestTrajectories _trajectories = new TestTrajectories(_driveTrainSubsystem.CONSTRAINTS);

  public PathWithLL() {

    requires(_driveTrainSubsystem);

    addSequential(new FollowTrajectoryCommand(_trajectories.getTestTrajectory()), 2.1);

    addSequential(new RotateCommandLL(false));

  }
}
