/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.autonomous;

import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.PathArcSegment;
import org.frcteam2910.common.control.PathLineSegment;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

/**
 * Add your docs here.
 */
public class TestTrajectories {

    private final Trajectory testTrajectory;
    private static final double TEST_END_VELO = 7.5 * 12;
    private static final Rotation2 TEST_PATH_ROTATION = Rotation2.fromDegrees(90.0);
    private static final Rotation2 TEST_PATH_START_ROTATION  = Rotation2.ZERO;
    public static final int SUBDIVIDE_ITERATIONS = 8;

    public TestTrajectories(ITrajectoryConstraint... constraints){
        Path testPath = new Path(TEST_PATH_START_ROTATION);
        testPath.addSegment(
            new PathLineSegment(
                new Vector2(0, 0),
                new Vector2(0, 48)
            )
        );
        testPath.addSegment(
            new PathArcSegment(
                new Vector2(0, 48),
                new Vector2(36, 84),
                new Vector2(36, 48)
            ),
            TEST_PATH_ROTATION
        );

        testPath.subdivide(SUBDIVIDE_ITERATIONS);
        testTrajectory = new Trajectory(0.0, TEST_END_VELO, testPath, constraints);
        
    }
    public Trajectory getTestTrajectory(){
        return testTrajectory;
    }
}
