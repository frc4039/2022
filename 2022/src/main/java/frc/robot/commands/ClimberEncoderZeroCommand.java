package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberEncoderZeroCommand extends CommandBase {
    private final ClimberSubsystem m_climberSubsystem;
    

    public ClimberEncoderZeroCommand(ClimberSubsystem climberSubsystem) {
        m_climberSubsystem = climberSubsystem;

        addRequirements(m_climberSubsystem);
    }

    @Override
    public void initialize() {
        m_climberSubsystem.initiateClimb();
        m_climberSubsystem.climberDown();
    }

    @Override
    public void end(boolean interrupted) {
        m_climberSubsystem.stop();
        m_climberSubsystem.setEncodersZero();
    }

    @Override
    public boolean isFinished() {
        return m_climberSubsystem.getBottomLeftLimit() && m_climberSubsystem.getBottomRightLimit();
    }
}
