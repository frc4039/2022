/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX m_shooterMotor1;
  private final TalonFX m_shooterMotor2;

  private final DoubleSolenoid m_shooterHood;
 
  public int type = 0;

  public ShooterSubsystem() {

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
    
    m_shooterMotor1.configVoltageCompSaturation(11.0);
    m_shooterMotor2.configVoltageCompSaturation(11.0);
    m_shooterMotor1.enableVoltageCompensation(true);
    m_shooterMotor2.enableVoltageCompensation(true);

    m_shooterHood = new DoubleSolenoid(Constants.kPCMCANID, PneumaticsModuleType.CTREPCM, 2, 3);
  }

  public void shoot(double shooterRPM) {
    m_shooterMotor1.set(ControlMode.Velocity, shooterRPM * ShooterConstants.kShooterGearRatio * 2048 / 600.0);
  }

  private void fenderLowShot() {
    shoot(ShooterConstants.kfenderLowShotRPM);
  }

  private void fenderHighShot() {
    shoot(ShooterConstants.kfenderHighShotRPM);
  }

  private void limelightShot() {
    shoot(ShooterConstants.klimelightShotRPM);
  }

  public void shotType() {
    if(type == 0)
      fenderHighShot();
    else if(type == 1)
      fenderLowShot();
    else if (type == 2)
      limelightShot();
    else
      fenderHighShot();
  }

  public void stop() {
    m_shooterMotor1.set(ControlMode.PercentOutput, 0);
  }

  public void extendShooterHood() {
    m_shooterHood.set(DoubleSolenoid.Value.kReverse);
  }

  public void retractShooterHood() {
    m_shooterHood.set(DoubleSolenoid.Value.kForward);
  }

  public void neutralShooterHood() {
    m_shooterHood.set(DoubleSolenoid.Value.kOff);
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