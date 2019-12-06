package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.autonomous.TestTrajectories;
import com.swervedrivespecialties.exampleswerve.commands.FollowTrajectoryCommand;
import com.swervedrivespecialties.exampleswerve.commands.TranslateCommandLL;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frcteam2910.common.robot.subsystems.Drivetrain;
import org.frcteam2910.common.robot.subsystems.SubsystemManager;

public class Robot extends TimedRobot {
    /**
     * How often the control thread should run in seconds.
     * By default it runs every 5 milliseconds.
     */
    private static final double UPDATE_DT = 5.0e-3;

    private static final OI oi = OI.getInstance();
    private static final DrivetrainSubsystem _driveTrainSubsystem = DrivetrainSubsystem.getInstance();

    private Command autonomousCommand;
    private TestTrajectories trajectories = new TestTrajectories(DrivetrainSubsystem.CONSTRAINTS);

    private final SubsystemManager subsystemManager = new SubsystemManager(
            DrivetrainSubsystem.getInstance()
    );

    public static OI getOi() {
        return oi;
    }

    @Override
    public void robotInit() {
        subsystemManager.enableKinematicLoop(UPDATE_DT);
    }

    @Override
    public void robotPeriodic() {
        subsystemManager.outputToSmartDashboard();

        Scheduler.getInstance().run();
    }
    @Override
    public void autonomousInit(){
        if (autonomousCommand != null){
            autonomousCommand.cancel();
        }
        autonomousCommand = new FollowTrajectoryCommand(trajectories.getTestTrajectory());
        autonomousCommand.start();
    }

    @Override
    public void autonomousPeriodic(){
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        SmartDashboard.putBoolean("Following Trajectory?", !DrivetrainSubsystem.getInstance().getFollower().getCurrentTrajectory().isEmpty());
        SmartDashboard.putNumber("alpha", _driveTrainSubsystem.getAlpha());
        for (int ind = 0; ind < 6; ind++){
            SmartDashboard.putNumber( "LLArr " + Double.toString(ind) + ": ", _driveTrainSubsystem.getLLArray()[ind]);
        }
    }
}
