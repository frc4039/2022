package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {

  private final CANSparkMax m_feederMotor;
  private static DigitalInput m_BreakBeamIntake = new DigitalInput(FeederConstants.kBreakBeamIntakePort);
  private static DigitalInput m_BreakBeamPreShooter = new DigitalInput(FeederConstants.kBreakBeamPreShooterPort);
  private static DigitalInput m_BreakBeamLowerBall = new DigitalInput(FeederConstants.kBreakBeamLowerBall);
  private static DigitalInput m_BreakBeamUpperBall = new DigitalInput(FeederConstants.kBreakBeamUpperBall);

  private final NetworkTableEntry intakeBB;
  private final NetworkTableEntry lowerBallBB;
  private final NetworkTableEntry upperBallBB;
  private final NetworkTableEntry preShooterBB;

  public FeederSubsystem() {
    m_feederMotor = new CANSparkMax(FeederConstants.kFeederPort, MotorType.kBrushless);
    m_feederMotor.restoreFactoryDefaults();
    m_feederMotor.setInverted(FeederConstants.kFeederInversion);
    //unique prime number frame period to avoid concurrent calls
    //this is based on anecdotal evidence that unique primes lower CAN utilization more than all devices at max interval
    m_feederMotor.setControlFramePeriodMs(97);

    ShuffleboardTab tab = Shuffleboard.getTab("Driver Readout");
        
    intakeBB = tab.add("Intake BB", false)
            .withPosition(5, 2)
            .withSize(1, 1)
            .getEntry();
    lowerBallBB = tab.add("Lower Ball BB", false)
            .withPosition(6, 2)
            .withSize(1, 1)
            .getEntry();
    upperBallBB = tab.add("Upper Ball BB", false)
            .withPosition(7, 2)
            .withSize(1, 1)
            .getEntry();
    preShooterBB = tab.add("PreShooter BB", false)
            .withPosition(8, 2)
            .withSize(1, 1)
            .getEntry();
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
    return (!m_BreakBeamLowerBall.get() || !m_BreakBeamIntake.get()) && !m_BreakBeamUpperBall.get();
  }

  @Override
  public void periodic() {
    intakeBB.setBoolean(getBreakBeamIntake());
    lowerBallBB.setBoolean(getBreakBeamLowerBall());
    upperBallBB.setBoolean(getBreakBeamUpperBall());
    preShooterBB.setBoolean(getBreakBeamPreShooter());
  }
}