/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;

/**
 * An example command that uses an example subsystem.
 */
public class ShootCommand extends CommandBase {
  private final FeederSubsystem m_feeder;
  private final ShooterSubsystem m_shooter;
  private final PreShooterSubsystem m_preShooter;
  private final LimelightSubsystem m_limelight;

  private double ShooterRPM = 0;
  private double PreShooterRPM = 0;
  private double RPMWindow = 0;
  private double preShooterRPMWindow = 0;
  private double feederPercent;

  /**
   * Creates a new Shoot Command.
   *
   * @param subsystem 
   */
  public ShootCommand(ShooterSubsystem shooter, PreShooterSubsystem preShooter, FeederSubsystem feeder, LimelightSubsystem limelight) {
    m_shooter = shooter;
    m_preShooter = preShooter;
    m_feeder = feeder;
    m_limelight = limelight;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_preShooter, m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.shotType();
    m_preShooter.preShotType();

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
      ShooterRPM = 8.5218 * m_limelight.getDistanceToTarget() + 1200;
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
      ShooterRPM = 8.5218 * m_limelight.getDistanceToTarget() + 1200;
      SmartDashboard.putNumber("Limelight RPM", ShooterRPM);
      m_shooter.shoot(ShooterRPM);
    }
    
    if ((m_shooter.returnCurrentRPM() > ShooterRPM * (1 - RPMWindow))
      && (m_shooter.returnCurrentRPM() < ShooterRPM * (1 + RPMWindow))
      && (m_preShooter.returnPreShooterCurrentRPM() > PreShooterRPM * (1 - preShooterRPMWindow))
      && (m_preShooter.returnPreShooterCurrentRPM() < PreShooterRPM * (1 + preShooterRPMWindow))
      )  {
            m_feeder.runFeeder(feederPercent);
    }
    /*
    else {
      m_feeder.stop();
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_preShooter.stop();
    m_feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
