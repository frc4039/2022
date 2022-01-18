/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX m_shooterMotor1;
  private final TalonFX m_shooterMotor2;

  public ShooterSubsystem() {
    TalonFXConfiguration shooterConfiguration = new TalonFXConfiguration();
    shooterConfiguration.slot0.kP = ShooterConstants.kShooterP;
    shooterConfiguration.slot0.kI = ShooterConstants.kShooterI;
    shooterConfiguration.slot0.kD = ShooterConstants.kShooterD;
    shooterConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    shooterConfiguration.supplyCurrLimit.currentLimit = ShooterConstants.kShooterCurrentLimit;
    shooterConfiguration.supplyCurrLimit.enable = true;
    shooterConfiguration.voltageCompSaturation = 11.5;

    m_shooterMotor1 = new TalonFX(ShooterConstants.kShooterMotorPort1);
    m_shooterMotor2 = new TalonFX(ShooterConstants.kShooterMotorPort2);
    m_shooterMotor1.configFactoryDefault(); 
    m_shooterMotor2.configFactoryDefault(); 
    m_shooterMotor1.setInverted(ShooterConstants.kShooterInversion1);
    m_shooterMotor2.setInverted(ShooterConstants.kShooterInversion2);
    m_shooterMotor1.configAllSettings(shooterConfiguration);
    m_shooterMotor2.configAllSettings(shooterConfiguration);
    m_shooterMotor1.enableVoltageCompensation(false);
    m_shooterMotor2.enableVoltageCompensation(false);
    m_shooterMotor2.follow(m_shooterMotor1);

    //m_shooterMotor1.config_kF(ShooterConstants.kPIDLoopIdx, ShooterConstants.kF, ShooterConstants.kTimeoutMs);
    //m_shooterMotor1.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.kP, ShooterConstants.kTimeoutMs);
  }

  public void shoot(double rpm) {
      m_shooterMotor1.set(ControlMode.Velocity, rpm * ShooterConstants.kShooterGearRatio * 2048 / 600.0);
  }

  public void stop() {
    m_shooterMotor1.set(ControlMode.PercentOutput, 0);
  }

  public void printShooterValues() {
    SmartDashboard.putNumber("Shooter RPM", returnCurrentRPM());
  }

  public double returnCurrentRPM() {
    return m_shooterMotor1.getSelectedSensorVelocity() * 600 / 2048 / ShooterConstants.kShooterGearRatio;
  }

  @Override
  public void periodic() {
    
  }
}