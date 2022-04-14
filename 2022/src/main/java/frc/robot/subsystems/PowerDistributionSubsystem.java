package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerDistributionSubsystem extends SubsystemBase {

    private final PowerDistribution m_PDP = new PowerDistribution(0, ModuleType.kCTRE);

    public double getVoltage() {
        return m_PDP.getVoltage();
        
    }

    public double getTotalCurrent() {
        return m_PDP.getTotalCurrent();
    }

    public double getChannelCurrent(int channel) {
        return m_PDP.getCurrent(channel);
    }

    public boolean getBrownout() {
        return m_PDP.getFaults().Brownout;
    }

}
