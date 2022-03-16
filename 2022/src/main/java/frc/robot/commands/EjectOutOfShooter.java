/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class EjectOutOfShooter extends CommandBase {
  private final FeederSubsystem m_feeder;
  private final ShooterSubsystem m_shooter;
  private final PreShooterSubsystem m_preShooter;

  private String shotType;
  private String preShotType;
  private boolean hasPassed = false;
  private boolean ejected = false;

  /**
   * Creates a new Shoot Command.
   *
   * @param subsystem 
   */
  public EjectOutOfShooter(ShooterSubsystem shooter, PreShooterSubsystem preShooter, FeederSubsystem feeder) {
    m_shooter = shooter;
    m_preShooter = preShooter;
    m_feeder = feeder;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_preShooter, m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      shotType = m_shooter.type;
      preShotType = m_preShooter.type;
      m_shooter.type = "low";
      m_preShooter.type = "low";
      m_feeder.runFeeder(FeederConstants.kFeederFeedPercent);
      m_shooter.shotType();
      m_preShooter.preShotType();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
