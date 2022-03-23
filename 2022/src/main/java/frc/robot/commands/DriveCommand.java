package frc.robot.commands;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.common.math.Vector2;
import frc.robot.Constants;
import frc.robot.common.input.Axis;

public class DriveCommand extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;
    private Axis forward;
    private Axis strafe;
    private Axis rotation;
    private SlewRateLimiter forwardRateLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter strafeRateLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(3);

    public DriveCommand(DrivetrainSubsystem drivetrain, Axis forward, Axis strafe, Axis rotation) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        drivetrainSubsystem = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrainSubsystem.drive(new Vector2(forwardRateLimiter.calculate(forward.get(true)), strafeRateLimiter.calculate(strafe.get(true))), rotationRateLimiter.calculate(rotation.get(true)) * Constants.kRotationScale, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(Vector2.ZERO, 0, false);
    }

}