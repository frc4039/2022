package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederSubsystem extends SubsystemBase {

  private final TalonSRX m_feederMotor;

  public FeederSubsystem() {
    m_feederMotor = new TalonSRX(ShooterConstants.kFeederPort);
    m_feederMotor.configFactoryDefault();
    m_feederMotor.setInverted(ShooterConstants.kFeederInversion);
  }

  
  public void stop() {
    m_feederMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void runFeeder() {
    m_feederMotor.set(ControlMode.PercentOutput, ShooterConstants.kFeederPercent);
  }

  public void reverseFeeder() {
    m_feederMotor.set(ControlMode.PercentOutput, -ShooterConstants.kFeederPercent);
  }

  public void feederSlowBackward() {
    m_feederMotor.set(ControlMode.PercentOutput, -0.1);
  }

  @Override
  public void periodic() {
    
  }
}