package frc.robot.common.autonomous;

import frc.robot.common.control.*;
import frc.robot.common.math.Rotation2;
import frc.robot.common.math.Vector2;

import java.io.IOException;
import java.util.Arrays;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;

    //Can you tell we'd just eaten lunch?
    private Trajectory noAuto;
    private Trajectory twoRightAuto1;
    private Trajectory twoRightAuto2;
    private Trajectory twoRightAuto3;
    private Trajectory twoLeftAuto1;
    private Trajectory twoLeftAuto2;
    private Trajectory fiveRightAuto1;
    private Trajectory fiveRightAuto2;
    private Trajectory fiveRightAuto3;
    private Trajectory fiveRightAuto4;
    private Trajectory fiveRightAuto5;
    private Trajectory threeRightAuto3;
    private Trajectory threeRightSlowAuto2;
    private Trajectory threeRightSlowAuto3;
    private Trajectory threeRightAuto4;
    private Trajectory fourLeftAuto1;
    private Trajectory fourLeftAuto2;
    private Trajectory fourLeftAuto3;
    private Trajectory fourLeftAuto4;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] normalConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        TrajectoryConstraint[] superSlowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        
        // Top speed
        superSlowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(1.5 * 12.0);
        // Acceleration speed
        superSlowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(1.5 * 12.0);

        // Top speed
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(4.0 * 12.0);
        // Acceleration speed
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);

        // Top speed
        normalConstraints[normalConstraints.length - 1] = new MaxVelocityConstraint(8.0 * 12.0);
        // Acceleration speed
        normalConstraints[normalConstraints.length - 2] = new MaxAccelerationConstraint(6.0 * 12.0);

        noAuto = new Trajectory(
            new SimplePathBuilder(new Vector2(0.0, 0.0), Rotation2.ZERO)
                .lineTo(new Vector2(0.0, 0.0), Rotation2.ZERO)
                .build(),
            trajectoryConstraints, SAMPLE_DISTANCE
        );

        twoRightAuto1 = new Trajectory(
            new SimplePathBuilder(new Vector2(-18, -88.5), Rotation2.fromDegrees(270.0))
                .lineTo(new Vector2(-18, -128.5), Rotation2.fromDegrees(270.0))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        twoRightAuto2 = new Trajectory(
            new SimplePathBuilder(new Vector2(-18, -128.5), Rotation2.fromDegrees(270.0))
                .lineTo(new Vector2(-18, -116.5), Rotation2.fromDegrees(81.0))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        twoRightAuto3 = new Trajectory(
            new SimplePathBuilder(new Vector2(-18, -116.5), Rotation2.fromDegrees(81.0))
                .lineTo(new Vector2(12, -116.5), Rotation2.fromDegrees(0))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        twoLeftAuto1 = new Trajectory(
            new SimplePathBuilder(new Vector2(-87.4, 42.1), Rotation2.fromDegrees(136.5))
                .lineTo(new Vector2(-116.5, 70.0), Rotation2.fromDegrees(136.5))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        twoLeftAuto2 = new Trajectory(
            new SimplePathBuilder(new Vector2(-116.5, 70), Rotation2.fromDegrees(136.5))
                .lineTo(new Vector2(-100.0, 70.0), Rotation2.fromDegrees(305.0))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        fiveRightAuto1 = new Trajectory(
            new SimplePathBuilder(new Vector2(-27.114, -93.212), Rotation2.fromDegrees(-88.5))
                .lineTo(new Vector2(-27.114, -133.206), Rotation2.fromDegrees(-90))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        fiveRightAuto2 = new Trajectory(
            new SimplePathBuilder(new Vector2(-27.114, -133.206), Rotation2.fromDegrees(-90))
                .lineTo(new Vector2(-164.6, -121.1), Rotation2.fromDegrees(40))
                .build(),
                normalConstraints, SAMPLE_DISTANCE
        );

        fiveRightAuto3 = new Trajectory(
            new SimplePathBuilder(new Vector2(-164.6, -121.1), Rotation2.fromDegrees(40))
                .lineTo(new Vector2(-138.1, -98.1), Rotation2.fromDegrees(40))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        fiveRightAuto4 = new Trajectory(
            new SimplePathBuilder(new Vector2(-138.1, -98.18), Rotation2.fromDegrees(40.7))
                .lineTo(new Vector2(-246.7, -98.7), Rotation2.fromDegrees(-133.75))
                .lineTo(new Vector2(-266.7, -116.7), Rotation2.fromDegrees(-133.75))
                .build(),
                normalConstraints, SAMPLE_DISTANCE
        );

        fiveRightAuto5 = new Trajectory(
            new SimplePathBuilder(new Vector2(-266.7, -116.7), Rotation2.fromDegrees(-133.75))
                .lineTo(new Vector2(-164.6, -121.1), Rotation2.fromDegrees(40.7))
                .build(),
                normalConstraints, SAMPLE_DISTANCE
        );

        threeRightAuto3 = new Trajectory(
            new SimplePathBuilder(new Vector2(-164.6, -121.1), Rotation2.fromDegrees(40))
                .lineTo(new Vector2(-146, -105), Rotation2.fromDegrees(40))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeRightSlowAuto2 = new Trajectory(
            new SimplePathBuilder(new Vector2(-27.114, -133.206), Rotation2.fromDegrees(-90))
                .lineTo(new Vector2(-50, -100), Rotation2.fromDegrees(63))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );
        
        threeRightSlowAuto3 = new Trajectory(
            new SimplePathBuilder(new Vector2(-50, -100), Rotation2.fromDegrees(63))
                .lineTo(new Vector2(-100, -100), Rotation2.fromDegrees(180))
                .lineTo(new Vector2(-50, -100), Rotation2.fromDegrees(63))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        threeRightAuto4 = new Trajectory(
            new SimplePathBuilder(new Vector2(-146, -105), Rotation2.fromDegrees(40))
                .lineTo(new Vector2(-138.1, -98.1), Rotation2.fromDegrees(40))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        fourLeftAuto1 = new Trajectory(
            new SimplePathBuilder(new Vector2(-87.408, 42.08), Rotation2.fromDegrees(136.5))
                .lineTo(new Vector2(-116.451, 70.019), Rotation2.fromDegrees(135))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        fourLeftAuto2 = new Trajectory(
            new SimplePathBuilder(new Vector2(-116.451, 70.019), Rotation2.fromDegrees(135))
                .lineTo(new Vector2(-158.627, 31.792), Rotation2.fromDegrees(-11.6395))
                .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        fourLeftAuto3 = new Trajectory(
            new SimplePathBuilder(new Vector2(-158.627, 31.792), Rotation2.fromDegrees(-11.6395))
            .lineTo(new Vector2(-258.7, -110.7), Rotation2.fromDegrees(-133.75))
            .lineTo(new Vector2(-278.7, -128.7), Rotation2.fromDegrees(-133.75))
                .build(),
                normalConstraints, SAMPLE_DISTANCE
        );

        fourLeftAuto4 = new Trajectory(
            new SimplePathBuilder(new Vector2(-278.7, -128.7), Rotation2.fromDegrees(-133.75))
                .lineTo(new Vector2(-158.627, 31.792), Rotation2.fromDegrees(-11.6395))
                .build(),
                normalConstraints, SAMPLE_DISTANCE
        );

    }

    public Trajectory getNoAuto() {
        return noAuto;
    }

    public Trajectory getTwoRightAuto1() {
        return twoRightAuto1;
    }

    public Trajectory getTwoRightAuto2() {
        return twoRightAuto2;
    }

    public Trajectory getTwoRightAuto3() {
        return twoRightAuto3;
    }

    public Trajectory getTwoLeftAuto1() {
        return twoLeftAuto1;
    }

    public Trajectory getTwoLeftAuto2() {
        return twoLeftAuto2;
    }

    public Trajectory getFiveRightAuto1() {
        return fiveRightAuto1;
    }

    public Trajectory getFiveRightAuto2() {
        return fiveRightAuto2;
    }

    public Trajectory getFiveRightAuto3() {
        return fiveRightAuto3;
    }

    public Trajectory getFiveRightAuto4() {
        return fiveRightAuto4;
    }

    public Trajectory getFiveRightAuto5() {
        return fiveRightAuto5;
    }

    public Trajectory getThreeRightAuto3() {
        return threeRightAuto3;
    }

    public Trajectory getThreeRightSlowAuto2() {
        return threeRightSlowAuto2;
    }

    public Trajectory getThreeRightSlowAuto3() {
        return threeRightSlowAuto3;
    }

    public Trajectory getThreeRightAuto4() {
        return threeRightAuto4;
    }

    public Trajectory getFourLeftAuto1() {
        return fourLeftAuto1;
    }

    public Trajectory getFourLeftAuto2() {
        return fourLeftAuto2;
    }

    public Trajectory getFourLeftAuto3() {
        return fourLeftAuto3;
    }

    public Trajectory getFourLeftAuto4() {
        return fourLeftAuto4;
    }
}