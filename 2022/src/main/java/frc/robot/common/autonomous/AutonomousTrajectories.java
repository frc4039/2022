package frc.robot.common.autonomous;

import frc.robot.common.control.*;
import frc.robot.common.io.PathReader;
import frc.robot.common.math.Rotation2;
import frc.robot.common.math.Vector2;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Arrays;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;

    private Trajectory tenFeetForwardTestAuto;
    private Trajectory tenFeetForwardSlowTestAuto;
    private Trajectory tenFeetRightTestAuto;
    private Trajectory tenFeetRightSlowTestAuto;
    private Trajectory tenFeetForwardRotatingTestAuto;
    private Trajectory tenFeetForwardRotatingSlowTestAuto;
    private Trajectory multipleStepsTestAuto;
    private Trajectory multipleStepsSlowTestAuto;
    private Trajectory pickUpNShootAuto1;
    private Trajectory pickUpNShootAuto2;


    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        
        // Top speed
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(4.0 * 12.0);
        // Acceleration speed
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);

        tenFeetForwardTestAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(120.0, 0.0), Rotation2.ZERO)
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        tenFeetForwardSlowTestAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(120.0, 0.0), Rotation2.ZERO)
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        tenFeetRightTestAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(0.0, 120.0), Rotation2.ZERO)
                        .build(),
                        trajectoryConstraints, SAMPLE_DISTANCE
        );

        tenFeetRightSlowTestAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(0.0, 120.0), Rotation2.ZERO)
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        tenFeetForwardRotatingTestAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(120.0, 0.0), Rotation2.fromDegrees(180.0))
                        .build(),
                        trajectoryConstraints, SAMPLE_DISTANCE
        );

        tenFeetForwardRotatingSlowTestAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(120.0, 0.0), Rotation2.fromDegrees(180.0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        multipleStepsTestAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(0.0, 120.0), Rotation2.fromDegrees(90.0))
                        .lineTo(new Vector2(120.0, 0), Rotation2.fromDegrees(180.0))
                        .lineTo(new Vector2(0.0, 0), Rotation2.fromDegrees(270.0))
                        .build(),
                        trajectoryConstraints, SAMPLE_DISTANCE
        );

        multipleStepsSlowTestAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(0.0, 120.0), Rotation2.fromDegrees(90.0))
                        .lineTo(new Vector2(120.0, 0), Rotation2.fromDegrees(180.0))
                        .lineTo(new Vector2(0.0, 0), Rotation2.fromDegrees(270.0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        pickUpNShootAuto1 = new Trajectory(
            new SimplePathBuilder(new Vector2(-18, -88.5), Rotation2.fromDegrees(270.0))
                .lineTo(new Vector2(-18, -128.5), Rotation2.fromDegrees(270.0))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        pickUpNShootAuto2 = new Trajectory(
            new SimplePathBuilder(new Vector2(-18, -128.5), Rotation2.fromDegrees(270.0))
                .lineTo(new Vector2(-6, -40.0), Rotation2.fromDegrees(68.0))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
    }

    public Trajectory getTenFeetForwardTestAuto() {
        return tenFeetForwardTestAuto;
    }

    public Trajectory getTenFeetForwardSlowTestAuto() {
        return tenFeetForwardSlowTestAuto;
    }
    
    public Trajectory getTenFeetRightTestAuto() {
        return tenFeetRightTestAuto;
    }
    
    public Trajectory getTenFeetRightSlowTestAuto() {
        return tenFeetRightSlowTestAuto;
    }
    
    public Trajectory getTenFeetForwardRotatingTestAuto() {
        return tenFeetForwardRotatingTestAuto;
    }
    
    public Trajectory getTenFeetForwardRotatingSlowTestAuto() {
        return tenFeetForwardRotatingSlowTestAuto;
    }
    
    public Trajectory getMultipleStepsTestAuto() {
        return multipleStepsTestAuto;
    }

    public Trajectory getMultipleStepsSlowTestAuto() {
        return multipleStepsSlowTestAuto;
    }

    public Trajectory getPickUpNShootAuto1() {
        return pickUpNShootAuto1;
    }

    public Trajectory getPickUpNShootAuto2() {
        return pickUpNShootAuto2;
    }
}