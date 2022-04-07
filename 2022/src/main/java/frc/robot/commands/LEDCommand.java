package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.AddressableLEDSubsystem;
import frc.robot.subsystems.FeederSubsystem;

public class LEDCommand extends CommandBase{
    
    private AddressableLEDSubsystem m_ledSubsystem;
    private FeederSubsystem m_feederSubsystem;
    private String m_mode;

    private int hue;
    private double hue2;

    public LEDCommand(AddressableLEDSubsystem ledSubsystem, FeederSubsystem feederSubsystem, String mode) {
        m_ledSubsystem = ledSubsystem;
        m_feederSubsystem = feederSubsystem;
        m_mode = mode;

        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        if (m_mode != "rainbow"
        && m_mode != "rainbow2"
        && m_mode != "rainbow3"
        && m_mode != "off"
        && m_mode != "red"
        && m_mode != "green"
        && m_mode != "blue"
        && m_mode != "white"
        && m_mode != "cargo") {
            m_mode = "rainbow2";
        }
    }

    @Override
    public void execute() {
        if (m_mode == "rainbow") {
            m_ledSubsystem.setHSVAll(hue, LEDConstants.SATURATION, LEDConstants.BRIGHTNESS);
            hue ++;
            if (hue > 180) {
                hue -= 180;
            }
        } else if (m_mode == "rainbow2") {
            m_ledSubsystem.setHSVAll((int)Math.round(hue2), LEDConstants.SATURATION, LEDConstants.BRIGHTNESS);
            hue2 += LEDConstants.RAINBOW2_HUE_SPEED;
            if (hue2 > 180) {
                hue2 -= 180;
            }
        } else if (m_mode == "rainbow3") {
            int hueOffset = hue;
            for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
                int hue = i + hueOffset;
                while (hue > 180) {
                    hue -= 180;
                }
                m_ledSubsystem.setHSV(i, hue, LEDConstants.SATURATION, LEDConstants.BRIGHTNESS);
                hueOffset++;
            }
            hue ++;
            if (hue > 180) {
                hue -= 180;
            }
        } else if (m_mode == "off") {
            m_ledSubsystem.setAllOff();
        } else if (m_mode == "red") {
            m_ledSubsystem.setRedAll();
        } else if (m_mode == "green") {
            m_ledSubsystem.setGreenAll();
        } else if (m_mode == "blue") {
            m_ledSubsystem.setBlueAll();
        } else if (m_mode == "cargo") {
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
