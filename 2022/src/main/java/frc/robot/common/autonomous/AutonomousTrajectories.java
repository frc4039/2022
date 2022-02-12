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

    private Trajectory tenFeetTestAuto;
    private Trajectory tenFeetSlowTestAuto;
    private Trajectory tenFeetNBackTestAuto;
    private Trajectory tenFeetNBackSlowTestAuto;
    private Trajectory tenFeetNBackSpinTestAuto;
    private Trajectory tenFeetNBackSpinSlowTestAuto;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);

        tenFeetTestAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(60.0, 0.0), Rotation2.ZERO)
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        tenFeetSlowTestAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(0.0, 120.0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        tenFeetNBackTestAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(0.0, 120.0))
                        .lineTo(new Vector2(0.0, 0.0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        tenFeetNBackSlowTestAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(0.0, 120.0))
                        .lineTo(new Vector2(0.0, 0.0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        tenFeetNBackSpinTestAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(0.0, 120.0))
                        .lineTo(new Vector2(0.0, 0.0), Rotation2.fromDegrees(180.0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        tenFeetNBackSpinSlowTestAuto = new Trajectory(
                new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                        .lineTo(new Vector2(0.0, 120.0))
                        .lineTo(new Vector2(0.0, 0.0), Rotation2.fromDegrees(180.0))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
    }

    private Path getPath(String name) throws IOException {
        InputStream in = getClass().getClassLoader().getResourceAsStream(name);
        if (in == null) {
            throw new FileNotFoundException("Path file not found: " + name);
        }

        try (PathReader reader = new PathReader(new InputStreamReader(in))) {
            return reader.read();
        }
    }

    public Trajectory getTenFeetTestAuto() {
        return tenFeetTestAuto;
    }

    public Trajectory getTenFeetSlowTestAuto() {
        return tenFeetSlowTestAuto;
    }

    public Trajectory getTenFeetNBackTestAuto() {
        return tenFeetNBackTestAuto;
    }

    public Trajectory getTenFeetNBackSlowTestAuto() {
        return tenFeetNBackSlowTestAuto;
    }

    public Trajectory getTenFeetNBackSpinTestAuto() {
        return tenFeetNBackSpinTestAuto;
    }

    public Trajectory getTenFeetNBackSpinSlowTestAuto() {
        return tenFeetNBackSpinSlowTestAuto;
    }
}