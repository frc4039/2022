/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class ComboShootCommand extends CommandBase {
  private final ShooterSubsystem m_shooter;
  private final double m_rpm;



  /**
   * Creates a new Shoot Command.
   *
   * @param subsystem 
   */
  public ComboShootCommand(ShooterSubsystem shooter, double rpm) {
    m_shooter = shooter;
    m_rpm = rpm;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.shoot(m_rpm);
    m_shooter.runPreShooter();
    m_shooter.runFeeder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
