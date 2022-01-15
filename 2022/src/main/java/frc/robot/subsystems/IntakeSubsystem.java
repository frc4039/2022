/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final VictorSPX m_intakeMotor;

  private final DoubleSolenoid m_intakeSolenoid;

  public IntakeSubsystem() {
    m_intakeMotor = new VictorSPX(IntakeConstants.kIntakeMotorPort); 
    m_intakeMotor.configFactoryDefault(); 
    m_intakeMotor.setInverted(IntakeConstants.kIntakeInversion);

    m_intakeSolenoid = IntakeConstants.kIntakeSolenoid;
  }

  public void intake() {
    m_intakeMotor.set(ControlMode.PercentOutput, IntakeConstants.kIntakePercent);
  }

  public void outtake() {
    m_intakeMotor.set(ControlMode.PercentOutput, -IntakeConstants.kIntakePercent);
  }

  public void extendIntake() {
      m_intakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractIntake() {
      m_intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void neutralIntake() {
      m_intakeSolenoid.set(DoubleSolenoid.Value.kOff);
  }

  public void stop() {
    m_intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    
  }
}