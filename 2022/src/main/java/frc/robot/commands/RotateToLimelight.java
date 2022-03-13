package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.common.math.Vector2;
import frc.robot.Constants;
import frc.robot.common.control.PidConstants;
import frc.robot.common.control.PidController;
import frc.robot.common.input.Axis;

public class RotateToLimelight extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;
    private Axis forward;
    private Axis strafe;
    LimelightSubsystem limelightSubsystem;

    private PidController rotationController = new PidController(new PidConstants(0.035, 0.0, 0));

    public RotateToLimelight(DrivetrainSubsystem drivetrain, Axis forward, Axis strafe, LimelightSubsystem limelightSubsystem) {
        this.forward = forward;
        this.strafe = strafe;
        this.limelightSubsystem = limelightSubsystem;

        drivetrainSubsystem = drivetrain;

        rotationController.setInputRange(0.0, 2*Math.PI);
        rotationController.setContinuous(true);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        rotationController.reset();
    }

    @Override
    public void execute() {
        rotationController.setSetpoint(drivetrainSubsystem.getPose().rotation.toRadians() - Math.toRadians(limelightSubsystem.getHorzAngleToGoal()));
        double rotationOutput = rotationController.calculate(drivetrainSubsystem.getPose().rotation.toRadians(), 0.02);
        drivetrainSubsystem.drive(new Vector2(forward.get(true), strafe.get(true)), rotationOutput, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(Vector2.ZERO, 0, false);
    }

}