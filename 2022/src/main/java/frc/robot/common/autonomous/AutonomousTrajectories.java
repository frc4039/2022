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
    private Trajectory frostyAuto1;
    private Trajectory frostyAuto2;
    private Trajectory frostyAuto3;
    private Trajectory frostyAuto4;
    private Trajectory frostyAuto5;
    private Trajectory mcFlurryAuto1;
    private Trajectory mcFlurryAuto2;
    private Trajectory mcFlurryAuto3;
    private Trajectory mcFlurryAuto4;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] normalConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        
        // Top speed
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(4.0 * 12.0);
        // Acceleration speed
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);

        // Top speed
        normalConstraints[normalConstraints.length - 1] = new MaxVelocityConstraint(8.0 * 12.0);
        // Acceleration speed
        normalConstraints[normalConstraints.length - 2] = new MaxAccelerationConstraint(8.0 * 12.0);

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

        frostyAuto1 = new Trajectory(
            new SimplePathBuilder(new Vector2(-27.114, -93.212), Rotation2.fromDegrees(-88.5))
                .lineTo(new Vector2(-27.114, -133.206), Rotation2.fromDegrees(-90))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        frostyAuto2 = new Trajectory(
            new SimplePathBuilder(new Vector2(-27.114, -133.206), Rotation2.fromDegrees(-90))
                .lineTo(new Vector2(-124.607, -113.762), Rotation2.fromDegrees(90))
                .build(),
                normalConstraints, SAMPLE_DISTANCE
        );

        frostyAuto3 = new Trajectory(
            new SimplePathBuilder(new Vector2(-124.607, -113.762), Rotation2.fromDegrees(90))
                .lineTo(new Vector2(-126.853, -93.018), Rotation2.fromDegrees(42))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        frostyAuto4 = new Trajectory(
            new SimplePathBuilder(new Vector2(-126.853, -93.018), Rotation2.fromDegrees(42))
                .lineTo(new Vector2(-246.659, -107.732), Rotation2.fromDegrees(-133.75))
                .lineTo(new Vector2(-250.659, -117.732), Rotation2.fromDegrees(-133.75))
                .build(),
                normalConstraints, SAMPLE_DISTANCE
        );

        frostyAuto5 = new Trajectory(
            new SimplePathBuilder(new Vector2(-250.659, -117.732), Rotation2.fromDegrees(-133.75))
                .lineTo(new Vector2(-126.853, -93.018), Rotation2.fromDegrees(42))
                .build(),
                normalConstraints, SAMPLE_DISTANCE
        );

        mcFlurryAuto1 = new Trajectory(
            new SimplePathBuilder(new Vector2(-87.408, 42.08), Rotation2.fromDegrees(136.5))
                .lineTo(new Vector2(-116.451, 70.019), Rotation2.fromDegrees(135))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        mcFlurryAuto2 = new Trajectory(
            new SimplePathBuilder(new Vector2(-116.451, 70.019), Rotation2.fromDegrees(135))
                .lineTo(new Vector2(-158.627, 31.792), Rotation2.fromDegrees(-11.6395))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        mcFlurryAuto3 = new Trajectory(
            new SimplePathBuilder(new Vector2(-158.627, 31.792), Rotation2.fromDegrees(-11.6395))
                .lineTo(new Vector2(-268.659, -129.732), Rotation2.fromDegrees(-133.75))
                .build(),
                normalConstraints, SAMPLE_DISTANCE
        );

        mcFlurryAuto4 = new Trajectory(
            new SimplePathBuilder(new Vector2(-268.659, -129.732), Rotation2.fromDegrees(-133.75))
                .lineTo(new Vector2(-158.627, 31.792), Rotation2.fromDegrees(-11.6395))
                .build(),
                normalConstraints, SAMPLE_DISTANCE
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

    public Trajectory getFrostyAuto1() {
        return frostyAuto1;
    }

    public Trajectory getFrostyAuto2() {
        return frostyAuto2;
    }

    public Trajectory getFrostyAuto3() {
        return frostyAuto3;
    }

    public Trajectory getFrostyAuto4() {
        return frostyAuto4;
    }

    public Trajectory getFrostyAuto5() {
        return frostyAuto5;
    }

    public Trajectory getMcFlurryAuto1() {
        return mcFlurryAuto1;
    }

    public Trajectory getMcFlurryAuto2() {
        return mcFlurryAuto2;
    }

    public Trajectory getMcFlurryAuto3() {
        return mcFlurryAuto3;
    }

    public Trajectory getMcFlurryAuto4() {
        return mcFlurryAuto4;
    }
}