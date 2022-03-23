package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberExtendCommand extends CommandBase {
    private final ClimberSubsystem m_climberSubsystem;
    

    public ClimberExtendCommand(ClimberSubsystem climberSubsystem) {
        m_climberSubsystem = climberSubsystem;

        addRequirements(m_climberSubsystem);
    }

    @Override
    public void initialize() {
       
    }

    @Override
    public void execute() {
        m_climberSubsystem.climberExtend();
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
