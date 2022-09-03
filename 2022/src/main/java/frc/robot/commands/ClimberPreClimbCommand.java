package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberPreClimbCommand extends CommandBase {
    private final ClimberSubsystem m_climberSubsystem;
    

    public ClimberPreClimbCommand(ClimberSubsystem climberSubsystem) {
        m_climberSubsystem = climberSubsystem;

        addRequirements(m_climberSubsystem);
    }

    @Override
    public void initialize() {
       m_climberSubsystem.climberUp();
    }

    @Override
    public void end(boolean interrupted) {
        m_climberSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return m_climberSubsystem.getClimberLeftEncoder() > ClimberConstants.kLeftClimberPreClimb && m_climberSubsystem.getClimberRightEncoder() > ClimberConstants.kRightClimberPreClimb;
    }
}
