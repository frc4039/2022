/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonSRX m_shooterMotor1;
  private final TalonSRX m_shooterMotor2;

  private final CANSparkMax m_preShooterMotor;
  private final SparkMaxPIDController m_preShooterPID;
  private final RelativeEncoder m_preShooterEncoder;
 
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

    m_shooterMotor1 = new TalonSRX(ShooterConstants.kShooterMotorPort1);
    m_shooterMotor2 = new TalonSRX(ShooterConstants.kShooterMotorPort2);

    m_shooterMotor1.configFactoryDefault(); 
    m_shooterMotor2.configFactoryDefault(); 

    m_shooterMotor1.setInverted(ShooterConstants.kShooterInversion1);
    m_shooterMotor2.setInverted(ShooterConstants.kShooterInversion2);

    m_shooterMotor1.setSensorPhase(ShooterConstants.kSensorInversion);

    m_shooterMotor2.follow(m_shooterMotor1);

    m_shooterMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
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
  
    m_preShooterMotor = new CANSparkMax(ShooterConstants.kPreShooterPort, MotorType.kBrushless);
    m_preShooterMotor.restoreFactoryDefaults();
    m_preShooterMotor.setInverted(ShooterConstants.kPreShooterInversion);
    m_preShooterPID = m_preShooterMotor.getPIDController();
    m_preShooterEncoder = m_preShooterMotor.getEncoder();
    m_preShooterPID.setP(ShooterConstants.kPreShooterP);
    m_preShooterPID.setI(ShooterConstants.kPreShooterI);
    m_preShooterPID.setD(ShooterConstants.kPreShooterD);
    m_preShooterPID.setFF(ShooterConstants.kPreShooterF);
    m_preShooterPID.setIZone(ShooterConstants.kPreShooterIZ);
  }

  public void shoot(double rpm) {
      m_shooterMotor1.set(ControlMode.Velocity, rpm * ShooterConstants.kShooterGearRatio * 4096 / 600.0);
  }

  public void stop() {
    m_shooterMotor1.set(ControlMode.PercentOutput, 0);
    m_preShooterMotor.set(0.0);
  }

  public void shooterSlowForward() {
    m_shooterMotor1.set(ControlMode.PercentOutput, 0.3);
  }

  public void shooterSlowBackward() {
    m_shooterMotor1.set(ControlMode.PercentOutput, -0.1);
  }

  public void printShooterValues() {
    SmartDashboard.putNumber("Shooter RPM", returnCurrentRPM());
  }

  public double returnCurrentRPM() {
    return m_shooterMotor1.getSelectedSensorVelocity() * 600 / 4096 / ShooterConstants.kShooterGearRatio;
  }

  public double returnPreShooterCurrentRPM() {
    return m_preShooterEncoder.getVelocity() / ShooterConstants.kPreShooterGearRatio;
  }

  public void runPreShooter(double rpm) {
    m_preShooterPID.setReference(rpm * ShooterConstants.kPreShooterGearRatio, CANSparkMax.ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    
  }
}