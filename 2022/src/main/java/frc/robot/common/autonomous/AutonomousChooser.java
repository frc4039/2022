package frc.robot.common.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.common.control.Trajectory;
import frc.robot.common.math.RigidTransform2;
import frc.robot.common.math.Rotation2;

public class AutonomousChooser {
    private final AutonomousTrajectories trajectories;

    private SendableChooser<AutonomousMode> autonomousModeChooser = new SendableChooser<>();

    public AutonomousChooser(AutonomousTrajectories trajectories) {
        this.trajectories = trajectories;

        autonomousModeChooser.setDefaultOption("10 Feet", AutonomousMode.TEN_FEET_TEST);
        autonomousModeChooser.addOption("10 Feet Slow", AutonomousMode.TEN_FEET_SLOW_TEST);
        autonomousModeChooser.addOption("10 Feet & Back", AutonomousMode.TEN_FEEN_N_BACK_TEST);
        autonomousModeChooser.addOption("10 Feet & Back Slow", AutonomousMode.TEN_FEET_N_BACK_SLOW_TEST);
        autonomousModeChooser.addOption("10 Feet & Back Spin", AutonomousMode.TEN_FEET_N_BACK_SPIN_TEST);
        autonomousModeChooser.addOption("10 Feet & Back Spin Slow", AutonomousMode.TEN_FEET_N_BACK_SPIN_SLOW_TEST);
    }

    public SendableChooser<AutonomousMode> getAutonomousModeChooser() {
        return autonomousModeChooser;
    }

    private Command getTenFeetTestAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTenFeetTestAuto());
        follow(command, container, trajectories.getTenFeetTestAuto());

        return command;
    }

    private Command getTenFeetSlowTestAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTenFeetSlowTestAuto());
        follow(command, container, trajectories.getTenFeetSlowTestAuto());

        return command;
    }

    private Command getTenFeetNBackTestAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTenFeetNBackTestAuto());
        follow(command, container, trajectories.getTenFeetNBackTestAuto());

        return command;
    }

    private Command getTenFeetNBackSlowTestAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTenFeetNBackSlowTestAuto());
        follow(command, container, trajectories.getTenFeetNBackSlowTestAuto());

        return command;
    }

    private Command getTenFeetNBackSpinTestAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTenFeetNBackSpinTestAuto());
        follow(command, container, trajectories.getTenFeetNBackSpinTestAuto());

        return command;
    }

    private Command getTenFeetNBackSpinSlowTestAutoCommand(RobotContainer container) {
        SequentialCommandGroup command = new SequentialCommandGroup();

        resetRobotPose(command, container, trajectories.getTenFeetNBackSpinSlowTestAuto());
        follow(command, container, trajectories.getTenFeetNBackSpinSlowTestAuto());

        return command;
    }

    public Command getCommand(RobotContainer container) {
        switch (autonomousModeChooser.getSelected()) {
            case TEN_FEET_TEST:
                return getTenFeetTestAutoCommand(container);
            case TEN_FEET_SLOW_TEST:
                return getTenFeetSlowTestAutoCommand(container);
            case TEN_FEEN_N_BACK_TEST:
                return getTenFeetNBackTestAutoCommand(container);
            case TEN_FEET_N_BACK_SLOW_TEST:
                return getTenFeetNBackSlowTestAutoCommand(container);
            case TEN_FEET_N_BACK_SPIN_TEST:
                return getTenFeetNBackSpinTestAutoCommand(container);
            case TEN_FEET_N_BACK_SPIN_SLOW_TEST:
                return getTenFeetNBackSpinSlowTestAutoCommand(container);
        }

        return getTenFeetTestAutoCommand(container);
    }

    private void follow(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new FollowTrajectoryCommand(container.getDrivetrainSubsystem(), trajectory));
    }

    private void resetRobotPose(SequentialCommandGroup command, RobotContainer container, Trajectory trajectory) {
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetGyroAngle(Rotation2.ZERO)));
        command.addCommands(new InstantCommand(() -> container.getDrivetrainSubsystem().resetPose(
                new RigidTransform2(trajectory.calculate(0.0).getPathState().getPosition(), Rotation2.ZERO))));
    }

    private enum AutonomousMode {
        TEN_FEET_TEST,
        TEN_FEET_SLOW_TEST,
        TEN_FEEN_N_BACK_TEST,
        TEN_FEET_N_BACK_SLOW_TEST,
        TEN_FEET_N_BACK_SPIN_TEST,
        TEN_FEET_N_BACK_SPIN_SLOW_TEST,
    }
}