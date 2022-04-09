package frc.robot.commands;


import java.util.ResourceBundle.Control;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.common.math.Vector2;
import frc.robot.Constants;
import frc.robot.common.control.PidConstants;
import frc.robot.common.control.PidController;
import frc.robot.common.input.Axis;

public class DriveCommand extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;
    private Axis forward;
    private Axis strafe;
    private Axis rotationXAxis;
    private Axis rotationYAxis;
    private SlewRateLimiter forwardRateLimiter = new SlewRateLimiter(2);
    private SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(2);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(2);
    private PidController rotationController = new PidController(new PidConstants(0.035, 0.0, 0));
    private double lastSetPoint;

    public DriveCommand(DrivetrainSubsystem drivetrain, Axis forward, Axis strafe, Axis rotationXAxis, Axis rotationYAxis) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotationXAxis = rotationXAxis;
        this.rotationYAxis = rotationYAxis;

        rotationController.setInputRange(0.0, 2*Math.PI);
        rotationController.setContinuous(true);

        drivetrainSubsystem = drivetrain;

        addRequirements(drivetrain);
    }
    @Override
    public void initialize() {
        rotationController.reset();
        lastSetPoint = drivetrainSubsystem.getPose().rotation.toRadians();
    }

    @Override
    public void execute() {
        if (Math.sqrt(Math.pow(rotationXAxis.get(false), 2) + Math.pow(rotationYAxis.get(false), 2)) > Constants.CONTROLLER_ROTATION_DEADBAND) {
            rotationController.setSetpoint(Math.atan2(-rotationYAxis.get(false), rotationXAxis.get(false)) + (Math.PI / 2));
            lastSetPoint = Math.atan2(-rotationYAxis.get(false), rotationXAxis.get(false)) + (Math.PI / 2);
        }
        else {
            rotationController.setSetpoint(lastSetPoint);
        }
        double rotationOutput = rotationController.calculate(drivetrainSubsystem.getPose().rotation.toRadians(), 0.02);
        drivetrainSubsystem.drive(new Vector2(forwardRateLimiter.calculate(forward.get(true)), strafeRateLimiter.calculate(strafe.get(true))), rotationLimiter.calculate(rotationOutput), true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(Vector2.ZERO, 0, false);
    }

}