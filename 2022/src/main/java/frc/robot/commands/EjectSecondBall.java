/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.common.util.InterpolatingDouble;
import frc.robot.common.util.InterpolatingTreeMap;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class EjectSecondBall extends CommandBase {
  private final FeederSubsystem m_feeder;
  private final ShooterSubsystem m_shooter;
  private final PreShooterSubsystem m_preShooter;
  private final LimelightSubsystem m_limelight;

  private String shotType;
  private String preShotType;
  private boolean hasPassed = false;
  private boolean ejected = false;

  private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shotProfile = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

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
  public EjectSecondBall(ShooterSubsystem shooter, PreShooterSubsystem preShooter, FeederSubsystem feeder, LimelightSubsystem limelight) {
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

    shotProfile.put(new InterpolatingDouble(105.0), new InterpolatingDouble(2100.0));
    shotProfile.put(new InterpolatingDouble(135.0), new InterpolatingDouble(2300.0));
    shotProfile.put(new InterpolatingDouble(185.0), new InterpolatingDouble(2700.0));
    shotProfile.put(new InterpolatingDouble(250.0), new InterpolatingDouble(3200.0));
    ShooterRPM = (double)(shotProfile.getInterpolated(new InterpolatingDouble(m_limelight.getDistanceToTarget())).value);
    
    PreShooterRPM = ShooterConstants.kpreShooterLimelightShotRPM;
    RPMWindow = ShooterConstants.klimelightShotRPMWindow;
    preShooterRPMWindow = ShooterConstants.kPreShooterlimelightShotRPMWindow;
    feederPercent = FeederConstants.kFeederLimelightShotPercent;

    
    shotType = m_shooter.type;
    preShotType = m_preShooter.type;
    m_shooter.type = "limelight";
    m_preShooter.type = "limelight";
    m_feeder.runFeeder(FeederConstants.kFeederFeedPercent);
    m_shooter.shotType();
    m_preShooter.preShotType();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ShooterRPM = (double)(shotProfile.getInterpolated(new InterpolatingDouble(m_limelight.getDistanceToTarget())).value);
    m_shooter.shoot(ShooterRPM);

    
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


    if (m_feeder.getBreakBeamPreShooter()){
        hasPassed = true;
    }
    if (hasPassed && !m_feeder.getBreakBeamPreShooter()){
        ejected = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_preShooter.stop();
    m_feeder.stop();

    m_shooter.type = shotType;
    m_preShooter.type = preShotType;
    hasPassed = false;
    ejected = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ejected;
  }
}
