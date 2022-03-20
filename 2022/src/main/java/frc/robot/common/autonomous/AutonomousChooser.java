package frc.robot.common.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
        autonomousModeChooser.addOption("(RIGHT) 3 Ball", AutonomousMode.THREE_RIGHT);
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
        setShotTypeLimelight(command, container);
        followAndIntake(command, container, trajectories.getTwoRightAuto1());
        followAndPreShoot(command, container, trajectories.getTwoRightAuto2());
        aimAndShoot(command, container, 3.0);
        follow(command, container, trajectories.getTwoRightAuto3());

        return command;
    }

    private SequentialCommandGroup getTwoLeftAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTwoLeftAuto1());
        setShotTypeLimelight(command, container);
        followAndIntake(command, container, trajectories.getTwoLeftAuto1());
        followAndPreShoot(command, container, trajectories.getTwoLeftAuto2());
        aimAndShoot(command, container, 3.0);

        return command;
    }

    private SequentialCommandGroup getFiveRightAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFiveRightAuto1());
        setShotTypeLimelight(command, container);
        followAndIntake(command, container, trajectories.getFiveRightAuto1());
        followAndPreShoot(command, container, trajectories.getFiveRightAuto2());
        
        followIntakeAndShoot(command, container, trajectories.getFiveRightAuto3(), -400);
        
        /*
        shoot(command,container, 1.0);
        followIntakeAndPreShoot(command, container, trajectories.getFiveRightAuto3());
        */

        shoot(command, container, 0.5);
        followAndIntake(command, container, trajectories.getFiveRightAuto4());
        intake(command, container, 1.0);
        followAndPreShoot(command, container, trajectories.getFiveRightAuto5());
        aimAndShoot(command, container, 2.0);
        
        return command;
    }

    private SequentialCommandGroup getFourLeftAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFourLeftAuto1());
        setShotTypeLimelight(command, container);
        followAndIntake(command, container, trajectories.getFourLeftAuto1());
        followAndPreShoot(command, container, trajectories.getFourLeftAuto2());
        shoot(command, container, 1.0);
        followAndIntake(command, container, trajectories.getFourLeftAuto3());
        intake(command, container, 1.0);
        followAndPreShoot(command, container, trajectories.getFourLeftAuto4());
        shoot(command, container, 2.0);

        return command;
    }

    private SequentialCommandGroup getThreeRightAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getFiveRightAuto1());
        setShotTypeLimelight(command, container);
        followAndIntake(command, container, trajectories.getFiveRightAuto1());
        followAndPreShoot(command, container, trajectories.getFiveRightAuto2());
        followAndPreShoot(command, container, trajectories.getThreeRightAuto3());
        shoot(command, container, 2.0);
        followAndIntake(command, container, trajectories.getThreeRightAuto4());
        shoot(command, container, 2.0);
        /*
        shoot(command,container, 1.0);
        followIntakeAndPreShoot(command, container, trajectories.getFiveRightAuto3());
        */

        
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
            case THREE_RIGHT:
                return getThreeRightAutoCommand(container);
            default:
                return getNoAutoCommand(container);
        }
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
            .deadlineWith(
                new FeederManagementCommand(container.getFeederSubsystem(), container)
            )
        );
    }

    private void followAndIntake(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
                .deadlineWith(new ParallelDeadlineGroup(
                        new IntakeCommand(container.getIntakeSubsystem()),
                        new FeederManagementCommand(container.getFeederSubsystem(), container))));
    }

    private void followIntakeAndPreShoot(SequentialCommandGroup command, RobotContainer container,Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
            .deadlineWith(
                new ParallelCommandGroup(
                    new PreShootCommand(container.getPreShooterSubsystem(), container.getShooterSubsystem(), container.getFeederSubsystem()),
                    new IntakeCommand(container.getIntakeSubsystem())
                )
            )
        );
    }

    private void followAndPreShoot(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
            .deadlineWith(
                new SequentialCommandGroup(
                    new FeederManagementCommand(container.getFeederSubsystem(), container).withTimeout(1.0),
                    new PreShootCommand(container.getPreShooterSubsystem(), container.getShooterSubsystem(), container.getFeederSubsystem())
                ) 
            )
        );
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
            new ChangeShotType(container.getShooterSubsystem(), container.getPreShooterSubsystem(), "limelight"),
            new InstantCommand(container.getLimelightSubsystem()::turnLEDOn, container.getLimelightSubsystem()));
    }

    private void intake(SequentialCommandGroup command, RobotContainer container, double timeout) {
        command.addCommands(new ParallelDeadlineGroup(
                new IntakeCommand(container.getIntakeSubsystem()),
                new FeederCommand(container.getFeederSubsystem(), -FeederConstants.kFeederFeedPercent))
                        .withTimeout(timeout));
    }

    private void aimAndShoot(SequentialCommandGroup command, RobotContainer container, double timeout) {
        command.addCommands(new ParallelRaceGroup(
            new RotateToLimelight(container.getDrivetrainSubsystem(), container.getDriveForwardAxis(), container.getDriveStrafeAxis(), container.getLimelightSubsystem()),
            new ShootCommand(container.getShooterSubsystem(), container.getPreShooterSubsystem(),
                container.getFeederSubsystem(), container.getLimelightSubsystem())
                        .withTimeout(timeout)));
    }

    private void followIntakeAndShoot(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory, double RPMChange) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory)
                .deadlineWith(new ParallelCommandGroup(
                        new IntakeCommand(container.getIntakeSubsystem()),
                        new MovingShootCommand(container.getShooterSubsystem(), container.getPreShooterSubsystem(),
                container.getFeederSubsystem(), container.getLimelightSubsystem(), RPMChange))));
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() ->
          container.getDrivetrainSubsystem().resetGyroAngle(trajectory.calculate(0.0).getPathState().getRotation().inverse())));
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
        THREE_RIGHT
    }
}