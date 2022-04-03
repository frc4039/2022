package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.common.math.RigidTransform2;
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
     private boolean updateOdometry;

    private PidController rotationController = new PidController(new PidConstants(0.04, 0.0, 0));

    public RotateToLimelight(DrivetrainSubsystem drivetrain, Axis forward, Axis strafe, LimelightSubsystem limelightSubsystem, boolean updateOdometry) {
        this.forward = forward;
        this.strafe = strafe;
        this.limelightSubsystem = limelightSubsystem;
        this.updateOdometry = updateOdometry;
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
        if (limelightSubsystem.getValidTarget()) {
            double angleToGoal = drivetrainSubsystem.getPose().rotation.toRadians() - Math.toRadians(limelightSubsystem.getHorzAngleToGoal());
            rotationController.setSetpoint(angleToGoal);
            
            if (updateOdometry)
                drivetrainSubsystem.resetPose(new RigidTransform2(new Vector2(Math.cos(angleToGoal + Math.PI) * limelightSubsystem.getDistanceToTarget(),
                Math.sin(angleToGoal + Math.PI) * limelightSubsystem.getDistanceToTarget()),
                drivetrainSubsystem.getPose().rotation));
        }
        else {
            double x = drivetrainSubsystem.getPose().translation.x;
            double y = drivetrainSubsystem.getPose().translation.y;

            rotationController.setSetpoint(Math.atan2(y, x));
        }
        
        double rotationOutput = rotationController.calculate(drivetrainSubsystem.getPose().rotation.toRadians(), 0.02);
        double feedForward = 0;

        if (rotationOutput > 0) {
            feedForward = Constants.LIMELIGHT_ROTATION_FEEDFORWARD;
        }
        else if (rotationOutput < 0) {
            feedForward = -Constants.LIMELIGHT_ROTATION_FEEDFORWARD;
        }

        SmartDashboard.putNumber("rotation output", rotationOutput + feedForward);
        drivetrainSubsystem.drive(new Vector2(forward.get(true), strafe.get(true)), rotationOutput + feedForward, true);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(Vector2.ZERO, 0, false);
    }

}