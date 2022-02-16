/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX m_intakeMotor;

  //private final DoubleSolenoid m_intakeSolenoid; 

  public IntakeSubsystem() {
    m_intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorPort);
    m_intakeMotor.configFactoryDefault(); 
    m_intakeMotor.setInverted(IntakeConstants.kIntakeInversion);

    //m_intakeSolenoid = new DoubleSolenoid(Constants.kPCMCANID, PneumaticsModuleType.CTREPCM, 0, 1);
  }

  public void intake(double speed) {
    m_intakeMotor.set(ControlMode.PercentOutput, speed);
    SmartDashboard.putNumber("Intake Speed", speed);
  }

  public void outtake(double speed) {
    m_intakeMotor.set(ControlMode.PercentOutput, -speed);
    SmartDashboard.putNumber("Intake Speed", -speed);
  }

  /*
  public void extendIntake() {
      m_intakeSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractIntake() {
      m_intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void neutralIntake() {
      m_intakeSolenoid.set(DoubleSolenoid.Value.kOff);
  }
  */
  
  public void stop() {
    m_intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {

  }
}