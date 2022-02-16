package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberUpCommand extends CommandBase {
    private final ClimberSubsystem m_climberSubsystem;
    

    public ClimberUpCommand(ClimberSubsystem climberSubsystem) {
        m_climberSubsystem = climberSubsystem;

        addRequirements(m_climberSubsystem);
    }

    @Override
    public void initialize() {
       m_climberSubsystem.climberUp();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        m_climberSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return m_climberSubsystem.getTopRightLimit() && m_climberSubsystem.getTopLeftLimit();
    }
}