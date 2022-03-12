package frc.robot.common.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.FeederConstants;
import frc.robot.commands.*;
import frc.robot.common.control.Trajectory;
import frc.robot.common.math.RigidTransform2;
public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("SELECT AUTO", AutonomousMode.NONE);
        autonomousModeChooser.addOption("(RIGHT) 2 Ball", AutonomousMode.TWO_RIGHT);
        autonomousModeChooser.addOption("(LEFT) 2 Ball", AutonomousMode.TWO_LEFT);
        autonomousModeChooser.addOption("(RIGHT/HP) 5 Ball", AutonomousMode.FIVE_RIGHT);
        autonomousModeChooser.addOption("(LEFT/HP) 4 Ball", AutonomousMode.FOUR_LEFT);
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }

    private SequentialCommandGroup getNoAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getNoAuto());

        return command;
    }

    private SequentialCommandGroup getTwoRightAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTwoRightAuto1());
        setShotTypeHigh(command, container);
        followAndIntake(command, container, trajectories.getTwoRightAuto1());
        followAndPreShoot(command, container, trajectories.getTwoRightAuto2());
        shoot(command, container, 10.0);

        return command;
    }

    private SequentialCommandGroup getTwoLeftAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTwoLeftAuto1());
        setShotTypeHigh(command, container);
        followAndIntake(command, container, trajectories.getTwoLeftAuto1());
        followAndPreShoot(command, container, trajectories.getTwoLeftAuto2());
        shoot(command, container, 10.0);

        return command;
    }

    private SequentialCommandGroup getFiveRightAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFiveRightAuto1());
        setShotTypeLimelight(command, container);
        followAndIntake(command, container, trajectories.getFiveRightAuto1());
        follow(command, container, trajectories.getFiveRightAuto2());
        shoot(command,container, 2.5);
        followIntakeAndPreShoot(command, container, trajectories.getFiveRightAuto3());
        shoot(command, container, 2.5);
        /*
        followAndIntake(command, container, trajectories.getFiveRightAuto4());
        intake(command, container, 2.0);
        followAndPreShoot(command, container, trajectories.getFiveRightAuto5());
        shoot(command, container, 2.0);
        */
        return command;
    }

    private SequentialCommandGroup getFourLeftAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFourLeftAuto1());
        setShotTypeLimelight(command, container);
        followAndIntake(command, container, trajectories.getFourLeftAuto1());
        follow(command, container, trajectories.getFourLeftAuto2());
        shoot(command, container, 2.0);
        follow(command, container, trajectories.getFourLeftAuto3());
        intake(command, container, 2.0);
        follow(command, container, trajectories.getFourLeftAuto4());
        shoot(command, container, 2.0);

        return command;
    }

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case TWO_RIGHT:
                return getTwoRightAutoCommand(container);
            case TWO_LEFT:
                return getTwoLeftAutoCommand(container);
            case FIVE_RIGHT:
                return getFiveRightAutoCommand(container);
            case FOUR_LEFT:
                return getFourLeftAutoCommand(container);
            default:
                return getNoAutoCommand(container);
        }
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
            .deadlineWith(
                new FeederManagementCommand(container.getFeederSubsystem())
            )
        );
    }

    private void followAndIntake(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
                .deadlineWith(new ParallelDeadlineGroup(
                        new IntakeCommand(container.getIntakeSubsystem()),
                        new FeederManagementCommand(container.getFeederSubsystem()))));
    }

    private void followIntakeAndPreShoot(SequentialCommandGroup command, RobotContainer container,
            Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
                .deadlineWith(
                    new ParallelCommandGroup(
                        new PreShootCommand(container.getPreShooterSubsystem(), container.getShooterSubsystem(), container.getFeederSubsystem()),
                        new IntakeCommand(container.getIntakeSubsystem())
                    )
                ));
    }

    private void followAndPreShoot(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
                .deadlineWith(
                    new PreShootCommand(container.getPreShooterSubsystem(), container.getShooterSubsystem(), container.getFeederSubsystem())
                ));
    }

    private void shoot(SequentialCommandGroup command, RobotContainer container, double timeout) {
        command.addCommands(new ShootCommand(container.getShooterSubsystem(), container.getPreShooterSubsystem(),
                container.getFeederSubsystem(), container.getLimelightSubsystem())
                        .withTimeout(timeout));
    }

    private void setShotTypeHigh(SequentialCommandGroup command, RobotContainer container) {
        command.addCommands(
            new ShooterHoodExtend(container.getShooterSubsystem()),
            new ChangeShotType(container.getShooterSubsystem(), container.getPreShooterSubsystem(), "high"));
    }
            
    private void setShotTypeLow(SequentialCommandGroup command, RobotContainer container) {
        command.addCommands(
            new ShooterHoodRetract(container.getShooterSubsystem()),
            new ChangeShotType(container.getShooterSubsystem(), container.getPreShooterSubsystem(), "low"));
    }
            
    private void setShotTypeLimelight(SequentialCommandGroup command, RobotContainer container) {
        command.addCommands(
            new ShooterHoodRetract(container.getShooterSubsystem()),
            new ChangeShotType(container.getShooterSubsystem(), container.getPreShooterSubsystem(), "limelight"));
    }

    private void intake(SequentialCommandGroup command, RobotContainer container, double timeout) {
        command.addCommands(new ParallelDeadlineGroup(
                new IntakeCommand(container.getIntakeSubsystem()),
                new FeederCommand(container.getFeederSubsystem(), -FeederConstants.kFeederFeedPercent))
                        .withTimeout(timeout));
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
        TWO_RIGHT,
        TWO_LEFT,
        FIVE_RIGHT,
        FOUR_LEFT,
    }
}