/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class EjectOutOfIntake extends CommandBase {
  private final FeederSubsystem m_feeder;
  private final IntakeSubsystem m_intake;

  private boolean hasPassed = false;
  private boolean ejected = false;

  /**
   * Creates a new Shoot Command.
   *
   * @param subsystem 
   */
  public EjectOutOfIntake(FeederSubsystem feeder, IntakeSubsystem intake) {
    m_intake = intake;
    m_feeder = feeder;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_feeder, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_intake.reverseIntake();
      m_intake.extendIntake();
      m_feeder.runFeederReverse(FeederConstants.kFeederFeedPercent);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (m_feeder.getBreakBeamIntake())
            hasPassed = true;
      else if (hasPassed && !m_feeder.getBreakBeamIntake())
            ejected = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
    m_intake.retractIntake();
    m_feeder.stop();
    hasPassed = false;
    ejected = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ejected;
  }
}
