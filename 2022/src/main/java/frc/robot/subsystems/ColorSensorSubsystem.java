package frc.robot.subsystems;

import frc.robot.common.drivers.PicoColorSensor;
import frc.robot.common.drivers.PicoColorSensor.RawColor;

public class ColorSensorSubsystem {
    
    private final PicoColorSensor m_colorSensor;
    
    public ColorSensorSubsystem() {
        m_colorSensor = new PicoColorSensor();
    }

    public boolean[] isSensorConnected() {
        return new boolean[] {m_colorSensor.isSensor0Connected(), m_colorSensor.isSensor1Connected()};
    }

    public int[] getProximity() {
        return new int[] {m_colorSensor.getProximity0(), m_colorSensor.getProximity1()};
    }

    public RawColor[] getRawColor() {
        return new RawColor[] {m_colorSensor.getRawColor0(), m_colorSensor.getRawColor1()};
    }

    public char[] getBallColor() {
        char[] ballColor = new char[] {0, 0};
        
        //TODO: Color sensor proximity threshold
        int[] proxThresh = new int[] {0, 0};

        for (int i = 0; i <= 1; i++) {
            if (isSensorConnected()[i] && getProximity()[i] < proxThresh[i]) {
                if (getRawColor()[i].blue > getRawColor()[i].red) {
                    ballColor[i] = 'B';
                } else if (getRawColor()[i].red > getRawColor()[i].blue) {
                    ballColor[i] = 'R';
                }
            }
        }

        return ballColor;
    }
}
