package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class AddressableLEDSubsystem extends SubsystemBase {
    
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private double hue;
    private double value;
    private double toggle = 0;
    
    public AddressableLEDSubsystem() {
        m_led = new AddressableLED(LEDConstants.LED_STRIP_PORT);
        m_ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_STRIP_LENGTH);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setRGB(int index, int r, int g, int b) {
        m_ledBuffer.setRGB(index, r, g, b);
    }

    public void setHSV(int index, int h, int s, int v) {
        m_ledBuffer.setHSV(index, h, s, v);
    }

    public void setRGBAll(int r, int g, int b) {
        for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
    }

    public void setHSVAll(int h, int s, int v) {
        for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            m_ledBuffer.setHSV(i, h, s, v);
        }
    }

    public void setRedAll() {
        for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            m_ledBuffer.setRGB(i, LEDConstants.BRIGHTNESS, 0, 0);
        }
    }

    public void setGreenAll() {
        for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            m_ledBuffer.setRGB(i, 0, LEDConstants.BRIGHTNESS, 0);
        }
    }

    public void setBlueAll() {
        for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            m_ledBuffer.setRGB(i, 0, 0, LEDConstants.BRIGHTNESS);
        }
    }

    public void setAllOff() {
        for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
    }

    public void setWhiteAll() {
        for (int i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            m_ledBuffer.setLED(i, Color.kWhite);
        }
    }

    public void pulseRainbow()
    {
        for (var i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++)
        {
            var varHue = (hue + (i * 180 / LEDConstants.LED_STRIP_LENGTH)) % 180;
            value += (float)LEDConstants.BRIGHTNESS / LEDConstants.LED_STRIP_LENGTH;
            value %= LEDConstants.BRIGHTNESS;
            setHSV(i, (int)Math.round(varHue), LEDConstants.SATURATION, (int)Math.round(value));
        }
        value -= 0.5;
        value %= LEDConstants.LED_STRIP_LENGTH;
        hue += 0.5;
        hue %= 180;
    }

    public void slowRainbow() {
        for (var i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            final var varHue = (hue + (i * 180 / LEDConstants.LED_STRIP_LENGTH)) % 180;
            setHSV(i, (int)Math.round(varHue), LEDConstants.SATURATION, LEDConstants.BRIGHTNESS);
        }

        hue += LEDConstants.RAINBOW_HUE_SPEED;
        hue %= 180;
    }

    public void rainbow() {
        for (var i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            final var varHue = (hue + (i * 180 / LEDConstants.LED_STRIP_LENGTH)) % 180;
            setHSV(i, (int)Math.round(varHue), LEDConstants.SATURATION, LEDConstants.BRIGHTNESS);
        }

        hue += LEDConstants.RAINBOW_HUE_SPEED*5;
        hue %= 180;
    }

    public void fastRainbow() {
        for (var i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            final var varHue = (hue + (i * 180 / LEDConstants.LED_STRIP_LENGTH)) % 180;
            setHSV(i, (int)Math.round(varHue), LEDConstants.SATURATION, LEDConstants.BRIGHTNESS);
        }

        hue += LEDConstants.RAINBOW_HUE_SPEED*15;
        hue %= 180;
    }

    public void progressBar(double progress, Color color) {
        int stripProgress = (int) (LEDConstants.LED_STRIP_LENGTH * progress);
        for (int i = 0; i < stripProgress; i++) {
            m_ledBuffer.setLED(i, color);
        }
        for (int i = LEDConstants.LED_STRIP_LENGTH - 1; i > stripProgress; i--) {
            m_ledBuffer.setLED(i, Color.kWhite);
        }
    }

        public void flashingHue(int hue) {
        for (var i = 0; i < LEDConstants.LED_STRIP_LENGTH; i++) {
            //HSV Hue is normally 0-360, WPILib Hue is 0-180
            //therefore <hue> below is divided by two, allowing Constants to be representative of traditional Hue values
            setHSV(i, hue/2, LEDConstants.SATURATION*(int)Math.round(toggle), LEDConstants.BRIGHTNESS*(int)Math.round(toggle));
        }

        toggle += 0.02 / LEDConstants.FLASHING_SPEED;
        toggle %= 1;
    }

    @Override
    public void periodic(){

        if (DriverStation.isDisabled()) {
            pulseRainbow();
        }

        m_led.setData(m_ledBuffer);
    }
}