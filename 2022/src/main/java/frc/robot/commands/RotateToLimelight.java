package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
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

    private PidController rotationController = new PidController(new PidConstants(0.04, 0.0, 0));

    private double angularVelocityToGoal;
    private double linearVelocityToGoal;

    private double lastCycleDistance;
    private double lastCycleAngle;
    

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

        double angle = Math.atan(drivetrainSubsystem.getPose().translation.y / drivetrainSubsystem.getPose().translation.x);
        angularVelocityToGoal = (angle - lastCycleAngle) / Constants.ROBOT_CYCLE_TIME;
        lastCycleAngle = angle;

        if (limelightSubsystem.getValidTarget()) {
            double distance = limelightSubsystem.getDistanceToTarget();
            linearVelocityToGoal = (distance - lastCycleDistance) / Constants.ROBOT_CYCLE_TIME;
            lastCycleDistance = distance;
        }
        else {
            double distance = Math.sqrt(Math.pow(drivetrainSubsystem.getPose().translation.x, 2) + Math.pow(drivetrainSubsystem.getPose().translation.y, 2));
            linearVelocityToGoal = (distance - lastCycleDistance) / Constants.ROBOT_CYCLE_TIME;
            lastCycleDistance = distance;
        }
        SmartDashboard.putNumber("Velocity To Goal", linearVelocityToGoal);
        SmartDashboard.putNumber("Angular Velocity To Goal", angularVelocityToGoal);

        if (limelightSubsystem.getValidTarget()) {
            double angleToGoal = drivetrainSubsystem.getPose().rotation.toRadians() - Math.toRadians(limelightSubsystem.getHorzAngleToGoal());

            rotationController.setSetpoint(angleToGoal);

            drivetrainSubsystem.resetPose(new RigidTransform2(new Vector2(Math.cos(angleToGoal + Math.PI) * limelightSubsystem.getDistanceToTarget(), Math.sin(angleToGoal + Math.PI) * limelightSubsystem.getDistanceToTarget()), drivetrainSubsystem.getPose().rotation));
        }
        else {
            double x = drivetrainSubsystem.getPose().translation.x;
            double y = drivetrainSubsystem.getPose().translation.y;

            if (x > 0 && y >= 0) {
                rotationController.setSetpoint(Math.atan(y / x) + Math.PI);
            }
            else if (x < 0 && y >= 0) {
                rotationController.setSetpoint(Math.atan(y / x));
            }
            else if (x < 0 && y <= 0) {
                rotationController.setSetpoint(Math.atan(y / x));
            }
            else if (x > 0 && y <= 0) {
                rotationController.setSetpoint(Math.atan(y / x) + Math.PI);
            }
            else if (x == 0){

                if (y > 0) {
                    rotationController.setSetpoint(-Math.PI / 2);
                }
                else {
                    rotationController.setSetpoint(Math.PI / 2);
                }
            }
            else {
                rotationController.setSetpoint(0);
            }
        }
        
        double rotationOutput = rotationController.calculate(drivetrainSubsystem.getPose().rotation.toRadians(), 0.02);
        drivetrainSubsystem.drive(new Vector2(forward.get(true), strafe.get(true)), rotationOutput, true);


    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(Vector2.ZERO, 0, false);
    }

}