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

    //Can you tell we'd just eaten lunch?
    private Trajectory noAuto;
    private Trajectory doubleJBCAuto1;
    private Trajectory doubleJBCAuto2;
    private Trajectory mcDoubleAuto1;
    private Trajectory mcDoubleAuto2;


    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        
        // Top speed
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(4.0 * 12.0);
        // Acceleration speed
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);

        noAuto = new Trajectory(
            new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                .lineTo(new Vector2(0.0, 0.0), Rotation2.ZERO)
                .build(),
            trajectoryConstraints, SAMPLE_DISTANCE
        );

        doubleJBCAuto1 = new Trajectory(
            new SimplePathBuilder(new Vector2(-18, -88.5), Rotation2.fromDegrees(270.0))
                .lineTo(new Vector2(-18, -128.5), Rotation2.fromDegrees(270.0))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        doubleJBCAuto2 = new Trajectory(
            new SimplePathBuilder(new Vector2(-18, -128.5), Rotation2.fromDegrees(270.0))
                .lineTo(new Vector2(-6, -40.0), Rotation2.fromDegrees(68.0))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        mcDoubleAuto1 = new Trajectory(
            new SimplePathBuilder(new Vector2(-91, 41.0), Rotation2.fromDegrees(137.0))
                .lineTo(new Vector2(-118.0, 64), Rotation2.fromDegrees(137.0))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        mcDoubleAuto2 = new Trajectory(
            new SimplePathBuilder(new Vector2(-118.0, 64), Rotation2.fromDegrees(137.0))
                .lineTo(new Vector2(-21.0, 11.0), Rotation2.fromDegrees(340.0))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

    }

    public Trajectory getNoAuto() {
        return noAuto;
    }

    public Trajectory getDoubleJBCAuto1() {
        return doubleJBCAuto1;
    }

    public Trajectory getDoubleJBCAuto2() {
        return doubleJBCAuto2;
    }

    public Trajectory getMcDoubleAuto1() {
        return mcDoubleAuto1;
    }

    public Trajectory getMcDoubleAuto2() {
        return mcDoubleAuto2;
    }
}