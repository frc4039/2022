package frc.robot.commands;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.AddressableLEDSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.FeederSubsystem;

public class LEDCommand extends CommandBase{
    
    private AddressableLEDSubsystem m_ledSubsystem;
    private FeederSubsystem m_feederSubsystem;
    private ClimberSubsystem m_climberSubsystem;

    public LEDCommand(AddressableLEDSubsystem ledSubsystem, FeederSubsystem feederSubsystem, ClimberSubsystem climberSubsystem) {
        m_ledSubsystem = ledSubsystem;
        m_feederSubsystem = feederSubsystem;
        m_climberSubsystem = climberSubsystem;

        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (DriverStation.isAutonomous()) {
            m_ledSubsystem.rainbow();
        } else if (m_climberSubsystem.getClimbeEnable()) {
            m_ledSubsystem.fastRainbow();
        } else if (DriverStation.getMatchTime() < 30) {
            if (m_feederSubsystem.getBothBallBreakBeams()) {
                m_ledSubsystem.flashingHue(LEDConstants.HUE_GREEN);
            }
            else if (m_feederSubsystem.getBreakBeamUpperBall()) {
                m_ledSubsystem.flashingHue(LEDConstants.HUE_BLUE);
            }
            else {
                m_ledSubsystem.flashingHue(LEDConstants.HUE_RED);
            }
        } else if (DriverStation.isEnabled()) {
            if (m_feederSubsystem.getBothBallBreakBeams()) {
                m_ledSubsystem.setGreenAll();
            }
            else if (m_feederSubsystem.getBreakBeamUpperBall()) {
                m_ledSubsystem.setBlueAll();
            }
            else {
                m_ledSubsystem.setRedAll();
            }
        } else {
            m_ledSubsystem.setWhiteAll();
        }
    }

    @Override
    public void end(boolean interupted) {
        m_ledSubsystem.setAllOff();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}