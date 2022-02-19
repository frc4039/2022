/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX m_shooterMotor1;
  private final TalonFX m_shooterMotor2;
  private final TalonFX m_preShooterMotor;

  public double ShooterRPM = 2800;
  public double PreShooterRPM = 2800;
 
  public ShooterSubsystem() {
    /*
    //Code for Talons configuration:

    TalonFXConfiguration shooterConfiguration = new TalonFXConfiguration();
    shooterConfiguration.slot0.kP = ShooterConstants.kShooterP;
    shooterConfiguration.slot0.kI = ShooterConstants.kShooterI;
    shooterConfiguration.slot0.kD = ShooterConstants.kShooterD;
    shooterConfiguration.slot0.kF = ShooterConstants.kShooterF;
    shooterConfiguration.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    shooterConfiguration.supplyCurrLimit.currentLimit = ShooterConstants.kShooterCurrentLimit;
    shooterConfiguration.supplyCurrLimit.enable = true;
    shooterConfiguration.voltageCompSaturation = 11.5;
    */

    m_shooterMotor1 = new TalonFX(ShooterConstants.kShooterMotorPort1);
    m_shooterMotor2 = new TalonFX(ShooterConstants.kShooterMotorPort2);

    m_shooterMotor1.configFactoryDefault(); 
    m_shooterMotor2.configFactoryDefault(); 

    m_shooterMotor1.setInverted(ShooterConstants.kShooterInversion1);
    m_shooterMotor2.setInverted(ShooterConstants.kShooterInversion2);

    m_shooterMotor1.setSensorPhase(ShooterConstants.kSensorInversion);

    m_shooterMotor2.follow(m_shooterMotor1);

    m_shooterMotor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    m_shooterMotor1.config_kF(0, ShooterConstants.kShooterF, 30);
    m_shooterMotor1.config_kP(0, ShooterConstants.kShooterP, 30);
    m_shooterMotor1.config_kI(0, ShooterConstants.kShooterI, 30);
    m_shooterMotor1.config_kD(0, ShooterConstants.kShooterD, 30);

    //m_shooterMotor1.configAllSettings(shooterConfiguration);
    //m_shooterMotor2.configAllSettings(shooterConfiguration);
    
    m_shooterMotor1.enableVoltageCompensation(false);
    m_shooterMotor2.enableVoltageCompensation(false);

    //m_shooterMotor1.config_kF(ShooterConstants.kPIDLoopIdx, ShooterConstants.kF, ShooterConstants.kTimeoutMs);
    //m_shooterMotor1.config_kP(ShooterConstants.kPIDLoopIdx, ShooterConstants.kP, ShooterConstants.kTimeoutMs);
  
    m_preShooterMotor = new TalonFX(ShooterConstants.kPreShooterPort);
    m_preShooterMotor.configFactoryDefault();
    m_preShooterMotor.setInverted(ShooterConstants.kPreShooterInversion);
    m_preShooterMotor.setSensorPhase(ShooterConstants.kPreShooterSensorInversion);
    m_preShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    m_preShooterMotor.config_kF(0, ShooterConstants.kPreShooterF, 30);
    m_preShooterMotor.config_kP(0, ShooterConstants.kPreShooterP, 30);
    m_preShooterMotor.config_kI(0, ShooterConstants.kPreShooterI, 30);
    m_preShooterMotor.config_kD(0, ShooterConstants.kPreShooterD, 30);
    m_preShooterMotor.enableVoltageCompensation(false);

  }

  public void shoot() {
   //m_shooterMotor1.set(ControlMode.Velocity, ShooterRPM * ShooterConstants.kShooterGearRatio * 2048 / 600.0);
  }

  public void stop() {
    m_shooterMotor1.set(ControlMode.PercentOutput, 0);
    m_preShooterMotor.set(ControlMode.PercentOutput, 0);
  }

  public void shooterSlowForward() {
    m_shooterMotor1.set(ControlMode.PercentOutput, 0.3);
  }

  public void shooterSlowBackward() {
    m_shooterMotor1.set(ControlMode.PercentOutput, -0.1);
    m_preShooterMotor.set(ControlMode.PercentOutput, -0.1);
  }

  public void printShooterValues() {
    SmartDashboard.putNumber("Shooter RPM", returnCurrentRPM());
    SmartDashboard.putNumber("PreShooter RPM", returnPreShooterCurrentRPM());
  }

  public double returnCurrentRPM() {
    return m_shooterMotor1.getSelectedSensorVelocity() * 600 / 2048 / ShooterConstants.kShooterGearRatio;
  }

  public double returnPreShooterCurrentRPM() {
    return m_preShooterMotor.getSelectedSensorVelocity() * 600 / 2048 / ShooterConstants.kPreShooterGearRatio;
  }

  public void runPreShooter() {
    m_preShooterMotor.set(ControlMode.Velocity, 2500 * ShooterConstants.kPreShooterGearRatio * 2048 / 600.0);
  //m_preShooterMotor.set(ControlMode.Velocity, PreShooterRPM * ShooterConstants.kPreShooterGearRatio * 2048 / 600.0);
  }

  @Override
  public void periodic() {
    
  }
}