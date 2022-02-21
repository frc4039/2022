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

        autonomousModeChooser.setDefaultOption("10 feet forward", AutonomousMode.TEN_FEET_FORWARD_TEST);
        autonomousModeChooser.addOption("10 feet forward slow", AutonomousMode.TEN_FEET_FORWARD_SLOW_TEST);
        autonomousModeChooser.addOption("10 feet right", AutonomousMode.TEN_FEET_RIGHT_TEST);
        autonomousModeChooser.addOption("10 feet right slow", AutonomousMode.TEN_FEET_RIGHT_SLOW_TEST);
        autonomousModeChooser.addOption("10 feet forward rotating", AutonomousMode.TEN_FEET_FORWARD_ROTATE_TEST);
        autonomousModeChooser.addOption("10 feet forward rotating slow", AutonomousMode.TEN_FEET_FORWARD_ROTATE_SLOW_TEST);
        autonomousModeChooser.addOption("Multiple steps", AutonomousMode.STEPS_TEST);
        autonomousModeChooser.addOption("Multiple steps slow", AutonomousMode.STEPS_SLOW_TEST);
        autonomousModeChooser.addOption("Pick up and shoot", AutonomousMode.PICK_UP_N_SHOOT);
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }

    private Command getTenFeetForwardTestAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTenFeetForwardTestAuto());
        follow(command, container, trajectories.getTenFeetForwardTestAuto());

        return command;
    }

    private Command getTenFeetForwardSlowTestAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTenFeetForwardSlowTestAuto());
        follow(command, container, trajectories.getTenFeetForwardSlowTestAuto());

        return command;
    }

    private Command getTenFeetRightTestAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTenFeetRightTestAuto());
        follow(command, container, trajectories.getTenFeetRightTestAuto());

        return command;
    }

    private Command getTenFeetRightSlowTestAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTenFeetRightSlowTestAuto());
        follow(command, container, trajectories.getTenFeetRightSlowTestAuto());

        return command;
    }

    private Command getTenFeetForwardRotatingTestAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTenFeetForwardRotatingTestAuto());
        follow(command, container, trajectories.getTenFeetForwardRotatingTestAuto());

        return command;
    }

    private Command getTenFeetForwardRotatingSlowTestAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTenFeetForwardRotatingSlowTestAuto());
        follow(command, container, trajectories.getTenFeetForwardRotatingSlowTestAuto());

        return command;
    }

    private Command getMultipleStepsTestAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getMultipleStepsTestAuto());
        follow(command, container, trajectories.getMultipleStepsTestAuto());

        return command;
    }

    private Command getMultipleStepsSlowTestAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getMultipleStepsSlowTestAuto());
        follow(command, container, trajectories.getMultipleStepsSlowTestAuto());

        return command;
    }

    private SequentialCommandGroup getPickUpNShootAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getPickUpNShootAuto1());
        followAndIntake(command, container, trajectories.getPickUpNShootAuto1());
        follow(command, container, trajectories.getPickUpNShootAuto2());
        shoot(command, container);

        return command;
    }

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case TEN_FEET_FORWARD_TEST:
                return getTenFeetForwardTestAutoCommand(container);
            case TEN_FEET_FORWARD_SLOW_TEST:
                return getTenFeetForwardSlowTestAutoCommand(container);
            case TEN_FEET_RIGHT_TEST:
                return getTenFeetRightTestAutoCommand(container);
            case TEN_FEET_RIGHT_SLOW_TEST:
                return getTenFeetRightSlowTestAutoCommand(container);
            case TEN_FEET_FORWARD_ROTATE_TEST:
                return getTenFeetForwardRotatingTestAutoCommand(container);
            case TEN_FEET_FORWARD_ROTATE_SLOW_TEST:
                return getTenFeetForwardRotatingSlowTestAutoCommand(container);
            case STEPS_TEST:
                return getMultipleStepsTestAutoCommand(container);
            case STEPS_SLOW_TEST:
                return getMultipleStepsSlowTestAutoCommand(container);
            case PICK_UP_N_SHOOT:
                return getPickUpNShootAutoCommand(container);
            default:
                return getTenFeetForwardTestAutoCommand(container);
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

    private void shoot(SequentialCommandGroup command, RobotContainer container) {
        command.addCommands(new ShootCommand(container.getShooterSubsystem(), container.getFeederSubsystem())
            .withTimeout(3.0));
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        //command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetGyroAngle(trajectory.calculate(0.0).getPathState().getRotation())));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(), trajectory.calculate(0.0).getPathState().getRotation()))));
    }

    private enum AutonomousMode {
        TEN_FEET_FORWARD_TEST,
        TEN_FEET_FORWARD_SLOW_TEST,
        TEN_FEET_RIGHT_TEST,
        TEN_FEET_RIGHT_SLOW_TEST,
        TEN_FEET_FORWARD_ROTATE_TEST,
        TEN_FEET_FORWARD_ROTATE_SLOW_TEST,
        STEPS_TEST,
        STEPS_SLOW_TEST,
        PICK_UP_N_SHOOT
    }
}