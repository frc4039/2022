package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class AddressableLEDSubsystem extends SubsystemBase {
    
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    
    public AddressableLEDSubsystem() {
        m_led = new AddressableLED(LEDConstants.LED_STRIP_PORT);
        m_led.setLength(LEDConstants.LED_STRIP_LENGTH);
    }

    public void setRGB(int index, int r, int g, int b) {
        m_ledBuffer.setRGB(index, r, g, b);
        m_led.setData(m_ledBuffer);
    }

    public void setHSV(int index, int h, int s, int v) {
        m_ledBuffer.setHSV(index, h, s, v);
        m_led.setData(m_ledBuffer);
    }

    public void setRGBAll(int r, int g, int b) {
        for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setHSVAll(int h, int s, int v) {
        for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            m_ledBuffer.setRGB(i, h, s, v);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setRedAll() {
        for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            m_ledBuffer.setRGB(i, LEDConstants.BRIGHTNESS, 0, 0);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setGreenAll() {
        for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            m_ledBuffer.setRGB(i, 0, LEDConstants.BRIGHTNESS, 0);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setBlueAll() {
        for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            m_ledBuffer.setRGB(i, 0, 0, LEDConstants.BRIGHTNESS);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setAllOff() {
        for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setWhiteAll() {
        for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        m_led.setData(m_ledBuffer);
    }
}
