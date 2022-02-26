package frc.robot.common.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.*;
import frc.robot.common.control.Trajectory;
import frc.robot.common.math.RigidTransform2;
import frc.robot.common.math.Rotation2;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("SELECT AUTO", AutonomousMode.NONE);
        autonomousModeChooser.addOption("Double JBC", AutonomousMode.DOUBLE_JBC);
        autonomousModeChooser.addOption("McDouble", AutonomousMode.MC_DOUBLE);
        autonomousModeChooser.addOption("Frosty", AutonomousMode.FROSTY);
        autonomousModeChooser.addOption("McFlurry1", AutonomousMode.MC_FLURRY1);
        autonomousModeChooser.addOption("McFlurry2", AutonomousMode.MC_FLURRY2);
        autonomousModeChooser.addOption("McFlurry3", AutonomousMode.MC_FLURRY3);
        autonomousModeChooser.addOption("McFlurry4", AutonomousMode.MC_FLURRY4);
        autonomousModeChooser.addOption("McFlurry5", AutonomousMode.MC_FLURRY5);
        autonomousModeChooser.addOption("McFlurry6", AutonomousMode.MC_FLURRY6);
        autonomousModeChooser.addOption("McFlurry7", AutonomousMode.MC_FLURRY7);
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }

    private SequentialCommandGroup getNoAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getNoAuto());

        return command;
    }

    private SequentialCommandGroup getDoubleJBCAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getDoubleJBCAuto1());
        followAndIntake(command, container, trajectories.getDoubleJBCAuto1());
        followAndPreShoot(command, container, trajectories.getDoubleJBCAuto2());
        shoot(command, container, 1.0);

        return command;
    }

    private SequentialCommandGroup getMcDoubleAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getMcDoubleAuto1());
        followAndIntake(command, container, trajectories.getMcDoubleAuto1());
        followAndPreShoot(command, container, trajectories.getMcDoubleAuto2());
        shoot(command, container, 1.0);

        return command;
    }

    private SequentialCommandGroup getFrostyAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFrostyAuto1());
        followAndIntake(command, container, trajectories.getFrostyAuto1());
        followAndIntake(command, container, trajectories.getFrostyAuto2());
        followIntakeAndPreShoot(command, container, trajectories.getFrostyAuto3());
        shoot(command, container, 2.5);
        followAndIntake(command, container, trajectories.getFrostyAuto4());
        intake(command, container, 2.0);
        followAndPreShoot(command, container, trajectories.getFrostyAuto5());
        shoot(command, container, 2.0);

        return command;
    }

    private SequentialCommandGroup getMcFlurryAutoCommand(RobotContainer container, int part) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        switch (part) {
            case 1:
                resetRobotPose(command, container, trajectories.getMcFlurryAuto1());
                followAndIntake(command, container, trajectories.getMcFlurryAuto1());
                break;
            case 2:
                resetRobotPose(command, container, trajectories.getMcFlurryAuto2());
                follow(command, container, trajectories.getMcFlurryAuto2());
                break;
            case 3:
                shoot(command, container, 2.0);
                break;
            case 4:
                resetRobotPose(command, container, trajectories.getMcFlurryAuto3());
                follow(command, container, trajectories.getMcFlurryAuto3());
                break;
            case 5:
                intake(command, container, 2.0);
                break;
            case 6:
                resetRobotPose(command, container, trajectories.getMcFlurryAuto4());
                follow(command, container, trajectories.getMcFlurryAuto4());
                break;
            case 7:
                shoot(command, container, 2.0);
                break;
        }

        return command;
    }

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case DOUBLE_JBC:
                return getDoubleJBCAutoCommand(container);
            case MC_DOUBLE:
                return getMcDoubleAutoCommand(container);
            case FROSTY:
                return getFrostyAutoCommand(container);
            case MC_FLURRY1:
                return getMcFlurryAutoCommand(container, 1);
            case MC_FLURRY2:
                return getMcFlurryAutoCommand(container, 2);
            case MC_FLURRY3:
                return getMcFlurryAutoCommand(container, 3);
            case MC_FLURRY4:
                return getMcFlurryAutoCommand(container, 4);
            case MC_FLURRY5:
                return getMcFlurryAutoCommand(container, 5);
            case MC_FLURRY6:
                return getMcFlurryAutoCommand(container, 6);
            case MC_FLURRY7:
                return getMcFlurryAutoCommand(container, 7);
            default:
                return getNoAutoCommand(container);
        }
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory));
    }

    private void followAndIntake(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
                .deadlineWith(new ParallelDeadlineGroup(
                    new IntakeCommand(container.getIntakeSubsystem()),
                    new FeederCommand(container.getFeederSubsystem(), ShooterConstants.kSlowFeederPercent)
                )));
    }

    private void followIntakeAndPreShoot(SequentialCommandGroup command, RobotContainer container,
            Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
                .deadlineWith(new ParallelDeadlineGroup(
                        new FeederCommand(container.getFeederSubsystem(), -ShooterConstants.kSlowFeederPercent)
                                .withTimeout(0.5),
                        new InstantCommand(container.getShooterSubsystem()::shooterSlowBackward,
                                container.getShooterSubsystem()),
                        new IntakeCommand(container.getIntakeSubsystem())))
                .andThen(
                        new InstantCommand(container.getShooterSubsystem()::shoot, container.getShooterSubsystem())));
    }

    private void followAndPreShoot(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
                .deadlineWith(new ParallelDeadlineGroup(
                        new FeederCommand(container.getFeederSubsystem(), -ShooterConstants.kSlowFeederPercent)
                                .withTimeout(0.5),
                        new InstantCommand(container.getShooterSubsystem()::shooterSlowBackward,
                                container.getShooterSubsystem())).andThen(
                                        new InstantCommand(container.getShooterSubsystem()::shoot,
                                                container.getShooterSubsystem()))));
    }

    private void shoot(SequentialCommandGroup command, RobotContainer container, Double timeout) {
        command.addCommands(new ShootCommand(container.getShooterSubsystem(), container.getFeederSubsystem())
                .withTimeout(timeout));
    }

    private void intake(SequentialCommandGroup command, RobotContainer container, Double timeout) {
        command.addCommands(new ParallelDeadlineGroup(
            new IntakeCommand(container.getIntakeSubsystem()),
            new FeederCommand(container.getFeederSubsystem(), ShooterConstants.kSlowFeederPercent)
        )
        .withTimeout(timeout)
        );
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        // command.addCommands(new InstantCommand(() ->
        // container.getDrivetrainSubsystem().resetGyroAngle(trajectory.calculate(0.0).getPathState().getRotation())));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(),
                        trajectory.calculate(0.0).getPathState().getRotation()))));
    }

    private enum AutonomousMode {
        NONE,
        DOUBLE_JBC,
        MC_DOUBLE,
        FROSTY,
        MC_FLURRY1,
        MC_FLURRY2,
        MC_FLURRY3,
        MC_FLURRY4,
        MC_FLURRY5,
        MC_FLURRY6,
        MC_FLURRY7,

    }
}