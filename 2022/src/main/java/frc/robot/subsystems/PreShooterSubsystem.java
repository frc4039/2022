/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class PreShooterSubsystem extends SubsystemBase {

  private final TalonFX m_preShooterMotor;

  public String type = "high";
 
  public PreShooterSubsystem() {
      
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

  public void stop() {
    m_preShooterMotor.set(ControlMode.PercentOutput, 0);
  }

  public double returnPreShooterCurrentRPM() {
    return m_preShooterMotor.getSelectedSensorVelocity() * 600 / 2048 / ShooterConstants.kPreShooterGearRatio;
  }

  public void runPreShooter(double preShooterRPM) {
    m_preShooterMotor.set(ControlMode.Velocity, preShooterRPM * ShooterConstants.kPreShooterGearRatio * 2048 / 600.0);
  }

  public void fenderLowPreShoot() {
    runPreShooter(ShooterConstants.kpreShooterFenderLowShotRPM);
  }

  public void fenderHighPreShoot() {
    runPreShooter(ShooterConstants.kpreShooterFenderHighShotRPM);
  }

  public void limelightPreShoot() {
    runPreShooter(ShooterConstants.kpreShooterLimelightShotRPM);
  }

  public void preShotType(){
    if(type.equals("high"))
      fenderHighPreShoot();
    else if(type.equals("low"))
      fenderLowPreShoot();
    else if (type.equals("limelight"))
      limelightPreShoot();
    else if (type.equals("hangar"))
      fenderHighPreShoot();
    else
      fenderHighPreShoot();
  }

  public void reversePreShooter() {
    m_preShooterMotor.set(ControlMode.Velocity, ShooterConstants.kpreShooterReverseRPM * ShooterConstants.kPreShooterGearRatio * 2048 / 600.0);
  }
}