package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class FeederSubsystem extends SubsystemBase {

  private final CANSparkMax m_feederMotor;

  public FeederSubsystem() {
    m_feederMotor = new CANSparkMax(ShooterConstants.kFeederPort, MotorType.kBrushless);
    m_feederMotor.restoreFactoryDefaults();
    m_feederMotor.setInverted(ShooterConstants.kFeederInversion);
  }

  
  public void stop() {
    m_feederMotor.set(0.0);
  }

  public void runFeeder(double speed) {
    m_feederMotor.set(speed);
  }

  public void reverseFeeder(double speed) {
    m_feederMotor.set(speed);
  }

  public void feederSlowBackward() {
    m_feederMotor.set(-0.1);
  }

  @Override
  public void periodic() {
    
  }
}