/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import frc.robot.Constants.ShooterConstants;

/**
 * An example command that uses an example subsystem.
 */
public class ShootCommand extends CommandBase {
  private final FeederSubsystem m_feeder;
  private final ShooterSubsystem m_shooter;
  private final double m_rpm;
  private final double m_preShooterRPM;



  /**
   * Creates a new Shoot Command.
   *
   * @param subsystem 
   */
  public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, double rpm, double preShooterRPM) {
    m_shooter = shooter;
    m_feeder = feeder;
    m_rpm = rpm;
    m_preShooterRPM = preShooterRPM;



    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.shoot(m_rpm);
    m_shooter.runPreShooter(m_preShooterRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((m_shooter.returnCurrentRPM() > m_rpm * ShooterConstants.kRPMWindow) && (m_shooter.returnPreShooterCurrentRPM() > m_preShooterRPM * ShooterConstants.kPreShooterRPMWindow)) {
      m_feeder.runFeeder();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
