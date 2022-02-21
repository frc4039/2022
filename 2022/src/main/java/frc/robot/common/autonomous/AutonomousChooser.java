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

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("SELECT AUTO", AutonomousMode.NONE);
        autonomousModeChooser.setDefaultOption("Double JBC", AutonomousMode.DOUBLE_JBC);
        autonomousModeChooser.addOption("McDouble", AutonomousMode.MC_DOUBLE);
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

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case DOUBLE_JBC:
                return getDoubleJBCAutoCommand(container);
            case MC_DOUBLE:
                return getMcDoubleAutoCommand(container);
            default:
                return getNoAutoCommand(container);
        }
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory));
    }

    private void followAndIntake(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
            .deadlineWith(new IntakeCommand(container.getIntakeSubsystem())));
    }

    private void followAndPreShoot(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
            .deadlineWith(new ParallelDeadlineGroup(
                new FeederCommand(container.getFeederSubsystem(), -ShooterConstants.kSlowFeederPercent).withTimeout(0.5),
                new InstantCommand(container.getShooterSubsystem()::shooterSlowBackward, container.getShooterSubsystem())
            ).andThen(
                new InstantCommand(container.getShooterSubsystem()::shoot, container.getShooterSubsystem())
            )));
    }

    private void shoot(SequentialCommandGroup command, RobotContainer container, Double timeout) {
        command.addCommands(new ShootCommand(container.getShooterSubsystem(), container.getFeederSubsystem())
            .withTimeout(timeout));
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        //command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetGyroAngle(trajectory.calculate(0.0).getPathState().getRotation())));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(), trajectory.calculate(0.0).getPathState().getRotation()))));
    }

    private enum AutonomousMode {
        NONE,
        DOUBLE_JBC,
        MC_DOUBLE,
    }
}