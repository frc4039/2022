/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.common.math.Rotation2;
import frc.robot.common.util.InterpolatingDouble;
import frc.robot.common.util.InterpolatingTreeMap;

/**
 * An example command that uses an example subsystem.
 */
public class ShootCommand extends CommandBase {
  private final FeederSubsystem m_feeder;
  private final ShooterSubsystem m_shooter;
  private final PreShooterSubsystem m_preShooter;
  private final LimelightSubsystem m_limelight;
  private final DrivetrainSubsystem m_drivetrain;

  private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shotProfile = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

  private double ShooterRPM = 0;
  private double PreShooterRPM = 0;
  private double RPMWindow = 0;
  private double preShooterRPMWindow = 0;
  private double feederPercent = 0;
  private double targetDistance = 0;
  /**
   * Creates a new Shoot Command.
   *
   * @param subsystem 
   */
  public ShootCommand(ShooterSubsystem shooter, PreShooterSubsystem preShooter, FeederSubsystem feeder, LimelightSubsystem limelight, DrivetrainSubsystem drivetrain) {
    m_shooter = shooter;
    m_preShooter = preShooter;
    m_feeder = feeder;
    m_limelight = limelight;
    m_drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_preShooter, m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.shotType();
    m_preShooter.preShotType();

    shotProfile.put(new InterpolatingDouble(ShooterConstants.kClosestKey), new InterpolatingDouble(ShooterConstants.kClosestValue));
    shotProfile.put(new InterpolatingDouble(ShooterConstants.kCloseKey), new InterpolatingDouble(ShooterConstants.kCloseValue));
    shotProfile.put(new InterpolatingDouble(ShooterConstants.kFarKey), new InterpolatingDouble(ShooterConstants.kFarValue));
    shotProfile.put(new InterpolatingDouble(ShooterConstants.kFarthestKey), new InterpolatingDouble(ShooterConstants.kFarthestValue));
    
    if (m_limelight.getValidTarget()) {
      targetDistance = m_limelight.getDistanceToTarget();
    }
    else {
      targetDistance = Math.sqrt(Math.pow(m_drivetrain.getPose().translation.x, 2) + Math.pow(m_drivetrain.getPose().translation.y, 2));
    }

    if (m_shooter.type == "high") {
      ShooterRPM = ShooterConstants.kfenderHighShotRPM;
      PreShooterRPM = ShooterConstants.kpreShooterFenderHighShotRPM;
      RPMWindow = ShooterConstants.kfenderHighShotRPMWindow;
      preShooterRPMWindow = ShooterConstants.kPreShooterFenderHighShotRPMWindow;
      feederPercent = FeederConstants.kFeederHighShotPercent;
    }
    else if (m_shooter.type == "low") {
      ShooterRPM = ShooterConstants.kfenderLowShotRPM;
      PreShooterRPM = ShooterConstants.kpreShooterFenderLowShotRPM;
      RPMWindow = ShooterConstants.kfenderLowShotRPMWindow;
      preShooterRPMWindow = ShooterConstants.kPreShooterFenderLowShotRPMWindow;
      feederPercent = FeederConstants.kFeederLowShotPercent;
    }
    else if (m_shooter.type == "limelight") {
      // limelightCommand = new RotateToLimelight(m_drivetrain, m_containter.getDriveForwardAxis(), m_containter.getDriveStrafeAxis(), m_limelight, false);
      // CommandScheduler.getInstance().schedule(limelightCommand);
      ShooterRPM = (double)(shotProfile.getInterpolated(new InterpolatingDouble(targetDistance)).value);
      PreShooterRPM = ShooterConstants.kpreShooterLimelightShotRPM;
      RPMWindow = ShooterConstants.klimelightShotRPMWindow;
      preShooterRPMWindow = ShooterConstants.kPreShooterlimelightShotRPMWindow;
      feederPercent = FeederConstants.kFeederLimelightShotPercent;
    }
    else {
      ShooterRPM = ShooterConstants.kfenderHighShotRPM;
      PreShooterRPM = ShooterConstants.kpreShooterFenderHighShotRPM;
      RPMWindow = ShooterConstants.kfenderHighShotRPMWindow;
      preShooterRPMWindow = ShooterConstants.kPreShooterFenderHighShotRPMWindow;
      feederPercent = FeederConstants.kFeederHighShotPercent;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(m_shooter.type == "limelight") {
      
      if (m_limelight.getValidTarget()) {
        targetDistance = m_limelight.getDistanceToTarget();
      }
      else {
        targetDistance = Math.sqrt(Math.pow(m_drivetrain.getPose().translation.x, 2) + Math.pow(m_drivetrain.getPose().translation.y, 2));
      }

      ShooterRPM = (double)(shotProfile.getInterpolated(new InterpolatingDouble(targetDistance)).value);
      // ShooterRPM = 8.5218 * m_limelight.getDistanceToTarget() + 1200;
      m_shooter.shoot(ShooterRPM);

      if ((m_shooter.returnCurrentRPM() > ShooterRPM * (1 - RPMWindow))
        && (m_shooter.returnCurrentRPM() < ShooterRPM * (1 + RPMWindow))
        && (m_preShooter.returnPreShooterCurrentRPM() > PreShooterRPM * (1 - preShooterRPMWindow))
        && (m_preShooter.returnPreShooterCurrentRPM() < PreShooterRPM * (1 + preShooterRPMWindow))
        && aimedAtTarget()
        && Math.abs(m_drivetrain.getAngularVelocity()) < ShooterConstants.kShotAngularVelocityLimit
        )  {
              m_feeder.runFeeder(feederPercent);
      } else if (m_feeder.getBreakBeamPreShooter()) {
        m_preShooter.reversePreShooter();
        m_feeder.runFeeder(-FeederConstants.kFeederFeedPercent);
      } else {
        m_preShooter.preShotType();
        m_feeder.stop();
      }
    }
    else {
      if ((m_shooter.returnCurrentRPM() > ShooterRPM * (1 - RPMWindow))
        && (m_shooter.returnCurrentRPM() < ShooterRPM * (1 + RPMWindow))
        && (m_preShooter.returnPreShooterCurrentRPM() > PreShooterRPM * (1 - preShooterRPMWindow))
        && (m_preShooter.returnPreShooterCurrentRPM() < PreShooterRPM * (1 + preShooterRPMWindow))
        )  {
              m_feeder.runFeeder(feederPercent);
      } else if (m_feeder.getBreakBeamPreShooter()) {
        m_preShooter.reversePreShooter();
        m_feeder.runFeeder(-FeederConstants.kFeederFeedPercent);
      } else {
        m_preShooter.preShotType();
        m_feeder.stop();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // limelightCommand.end(false);
    m_shooter.stop();
    m_preShooter.stop();
    m_feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean aimedAtTarget() {
    if (m_limelight.getValidTarget()) {
      return true;
    }
    else {
      double rotation = m_drivetrain.getPose().rotation.toDegrees();
      double toTarget = Math.toDegrees(Math.atan2(m_drivetrain.getPose().translation.y, m_drivetrain.getPose().translation.x));

      double angleDifference = Math.abs(rotation - toTarget);
      if (angleDifference > 180) {
        angleDifference -= 360;
      }

      return angleDifference < ShooterConstants.kAngleWindow;
    }
  }
}
