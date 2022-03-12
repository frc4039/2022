package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {

  private final CANSparkMax m_feederMotor;
  private static DigitalInput m_BreakBeamIntake;
  private static DigitalInput m_BreakBeamPreShooter;
  private static DigitalInput m_BreakBeamLowerBall;
  private static DigitalInput m_BreakBeamUpperBall;

  public FeederSubsystem() {
    m_feederMotor = new CANSparkMax(FeederConstants.kFeederPort, MotorType.kBrushless);
    m_feederMotor.restoreFactoryDefaults();
    m_feederMotor.setInverted(FeederConstants.kFeederInversion);

    m_BreakBeamIntake = new DigitalInput(FeederConstants.kBreakBeamIntakePort);
    m_BreakBeamPreShooter = new DigitalInput(FeederConstants.kBreakBeamPreShooterPort);
    m_BreakBeamLowerBall = new DigitalInput(FeederConstants.kBreakBeamLowerBall);
    m_BreakBeamUpperBall = new DigitalInput(FeederConstants.kBreakBeamUpperBall);
  }

  
  public void stop() {
    m_feederMotor.set(0.0);
  }

  public void runFeeder(double speed) {
    m_feederMotor.set(speed);
  }

  public void runFeederReverse(double speed) {
    m_feederMotor.set(-speed);
  }

  public boolean getBreakBeamIntake(){
    return !m_BreakBeamIntake.get();
  }

  public boolean getBreakBeamPreShooter(){
    return !m_BreakBeamPreShooter.get();
  }

  public boolean getBothBreakBeams(){
    return !m_BreakBeamIntake.get() && !m_BreakBeamPreShooter.get();
  }

  public boolean getBreakBeamLowerBall() {
    return !m_BreakBeamLowerBall.get();
  }

  public boolean getBreakBeamUpperBall() {
    return !m_BreakBeamUpperBall.get();
  }

  public boolean getBothBallBreakBeams(){
    return !m_BreakBeamLowerBall.get() && !m_BreakBeamUpperBall.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake BB", getBreakBeamIntake());
    SmartDashboard.putBoolean("PreShooter BB", getBreakBeamPreShooter());  
    SmartDashboard.putBoolean("Upper ball BB", getBreakBeamUpperBall());
    SmartDashboard.putBoolean("Lower ball BB", getBreakBeamLowerBall());  
  }
}