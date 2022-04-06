package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.AddressableLEDSubsystem;

public class LEDCommand extends CommandBase{
    
    private AddressableLEDSubsystem m_ledSubsystem;
    private String m_mode;

    private int hue;
    private double hue2;

    public LEDCommand(AddressableLEDSubsystem ledSubsystem, String mode) {
        m_ledSubsystem = ledSubsystem;
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
        && m_mode != "white") {
            m_mode = "rainbow2";
        }
    }

    @Override
    public void execute() {
        if (m_mode == "rainbow") {
            m_ledSubsystem.setHSVAll(hue, LEDConstants.BRIGHTNESS, LEDConstants.BRIGHTNESS);
            hue += 1;
            if (hue > 180) {
                hue -= 180;
            }
        } else if (m_mode == "rainbow2") {
            m_ledSubsystem.setHSVAll((int)Math.round(hue2), LEDConstants.BRIGHTNESS, LEDConstants.BRIGHTNESS);
            hue2 += LEDConstants.RAINBOW2_HUE_SPEED;
            if (hue2 > 180) {
                hue2 -= 180;
            }
        } else if (m_mode == "rainbow3") {
            int hueOffset = 0;
            for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
                int hue = i + hueOffset;
                if (hue > 180) {
                    hue -= 180;
                }
                m_ledSubsystem.setHSV(i, hue, LEDConstants.BRIGHTNESS, LEDConstants.BRIGHTNESS);
                hueOffset++;
            }
        } else if (m_mode == "off") {
            m_ledSubsystem.setRGBAll(0, 0, 0);
        } else if (m_mode == "red") {
            m_ledSubsystem.setRGBAll(LEDConstants.BRIGHTNESS, 0, 0);
        } else if (m_mode == "green") {
            m_ledSubsystem.setRGBAll(0, LEDConstants.BRIGHTNESS, 0);
        } else if (m_mode == "blue") {
            m_ledSubsystem.setRGBAll(0, 0, LEDConstants.BRIGHTNESS);
        } else {
            m_ledSubsystem.setRGBAll(LEDConstants.BRIGHTNESS, LEDConstants.BRIGHTNESS, LEDConstants.BRIGHTNESS);
        }
    }

    @Override
    public void end(boolean interupted) {
        m_ledSubsystem.setRGBAll(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
