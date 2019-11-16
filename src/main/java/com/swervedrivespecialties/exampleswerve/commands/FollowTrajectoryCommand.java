package com.swervedrivespecialties.exampleswerve.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Vector2;

import java.util.function.Supplier;

public class FollowTrajectoryCommand extends Command {
    private final Supplier<Trajectory> trajectorySupplier;

    private Trajectory trajectory;

    public FollowTrajectoryCommand(Trajectory trajectory) {
        this(() -> trajectory);
    }

    public FollowTrajectoryCommand(Supplier<Trajectory> trajectorySupplier) {
        this.trajectorySupplier = trajectorySupplier;

        requires(DrivetrainSubsystem.getInstance());
        this.setRunWhenDisabled(true);
    }

    @Override
    protected void initialize() {
        trajectory = trajectorySupplier.get();
        DrivetrainSubsystem.getInstance().resetKinematics(Vector2.ZERO, Timer.getFPGATimestamp());
        DrivetrainSubsystem.getInstance().getFollower().follow(trajectory);
        SmartDashboard.putBoolean("Running Follow Trajectory", true);
    }

    @Override
    protected void end() {
        DrivetrainSubsystem.getInstance().setSnapRotation(trajectory.calculateSegment(trajectory.getDuration()).rotation.toRadians());
        SmartDashboard.putBoolean("Running Following Trajectory", false);
    }

    @Override
    protected void interrupted() {
        end();
        DrivetrainSubsystem.getInstance().getFollower().cancel();
    }

    @Override
    protected boolean isFinished() {
        // Only finish when the trajectory is completed
        return DrivetrainSubsystem.getInstance().getFollower().getCurrentTrajectory().isEmpty();
    }
}