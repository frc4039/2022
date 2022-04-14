package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    
    private final TalonFX m_climberMotorRight;

    private final TalonFX m_climberMotorLeft;

    private final DoubleSolenoid m_climberSolenoid;

    private boolean enableClimb = false;
    private boolean climberExtended = false;

    private final DigitalInput leftBottomLimitSwitch;
    private final DigitalInput rightBottomLimitSwitch;
    private final DigitalInput leftTopBreakBeam;
    private final DigitalInput rightTopBreakBeam;

    
    private final NetworkTableEntry topLeftBB;
    private final NetworkTableEntry topRightBB;
    private final NetworkTableEntry bottomLeftLimit;
    private final NetworkTableEntry bottomRightLimit;
    private final NetworkTableEntry climberEnable;
    

    public ClimberSubsystem() {
        m_climberMotorRight = new TalonFX(ClimberConstants.kClimberMotorRightPort);
        m_climberMotorLeft = new TalonFX(ClimberConstants.kClimberMotorLeftPort);
        m_climberMotorRight.configFactoryDefault();
        m_climberMotorLeft.configFactoryDefault();
        m_climberMotorRight.setInverted(ClimberConstants.kClimberMotorRightInversion);
        m_climberMotorLeft.setInverted(ClimberConstants.kClimberMotorLeftInversion);
        m_climberMotorRight.setNeutralMode(NeutralMode.Brake);
        m_climberMotorLeft.setNeutralMode(NeutralMode.Brake);

        m_climberSolenoid = new DoubleSolenoid(Constants.kPCMCANID, PneumaticsModuleType.CTREPCM, 2, 3);
        
        m_climberMotorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_climberMotorRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        m_climberMotorLeft.getSensorCollection().setIntegratedSensorPosition(0, ClimberConstants.kTimeoutMs);
        m_climberMotorRight.getSensorCollection().setIntegratedSensorPosition(0, ClimberConstants.kTimeoutMs);


        //m_climberMotorLeft.configReverseSoftLimitThreshold(0);
        m_climberMotorLeft.configForwardSoftLimitThreshold(ClimberConstants.kFullyClimbedTicks);
        //m_climberMotorRight.configReverseSoftLimitThreshold(0);
        m_climberMotorRight.configForwardSoftLimitThreshold(ClimberConstants.kFullyClimbedTicks);
        //m_climberMotorLeft.configReverseSoftLimitEnable(true);
        m_climberMotorLeft.configForwardSoftLimitEnable(true);
        //m_climberMotorRight.configReverseSoftLimitEnable(true);
        m_climberMotorRight.configForwardSoftLimitEnable(true);
        
        m_climberMotorRight.config_kF(0, ClimberConstants.kClimberF, 30);
        m_climberMotorRight.config_kP(0, ClimberConstants.kClimberP, 30);
        m_climberMotorRight.config_kI(0, ClimberConstants.kClimberI, 30);
        m_climberMotorRight.config_kD(0, ClimberConstants.kClimberD, 30);

        m_climberMotorLeft.config_kF(0, ClimberConstants.kClimberF, 30);
        m_climberMotorLeft.config_kP(0, ClimberConstants.kClimberP, 30);
        m_climberMotorLeft.config_kI(0, ClimberConstants.kClimberI, 30);
        m_climberMotorLeft.config_kD(0, ClimberConstants.kClimberD, 30);

        //unique prime CAN status frame periods to avoid concurrent calls
        //this is based on anecdotal evidence that unique primes lower CAN utilization more than all devices at max interval
        //set climber motor status frames high at robot initialization, these are reset to 
        //default once the climb is initiated
        m_climberMotorLeft.setStatusFramePeriod(1, 229);
        m_climberMotorLeft.setStatusFramePeriod(2, 227);
        m_climberMotorRight.setStatusFramePeriod(1, 223);
        m_climberMotorRight.setStatusFramePeriod(2, 211);

        leftBottomLimitSwitch = new DigitalInput(ClimberConstants.kLeftBottomLimitSwitchPort);
        rightBottomLimitSwitch = new DigitalInput(ClimberConstants.kRightBottomLimitSwitchPort);
        leftTopBreakBeam = new DigitalInput(ClimberConstants.kLeftTopBreakBeamPort);
        rightTopBreakBeam = new DigitalInput(ClimberConstants.kRightTopBreakBeamPort);

        ShuffleboardTab tab = Shuffleboard.getTab("Driver Readout");
        
        topLeftBB = tab.add("Clmb TL BB", false)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        topRightBB = tab.add("Clmb TR BB", false)
                .withPosition(2, 1)
                .withSize(1, 1)
                .getEntry();
        bottomLeftLimit = tab.add("Clmb BL LS", false)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();
        bottomRightLimit = tab.add("Clmb BR LS", false)
                .withPosition(2, 2)
                .withSize(1, 1)
                .getEntry();
        climberEnable = tab.add("Climber Enabled", false)
                .withPosition(1, 1)
                .withSize(1, 2)
                .getEntry();
    }

    public void setClimberVelocityUp() {
        if(enableClimb) {
            if (getTopRightBB()) {
                m_climberMotorRight.set(ControlMode.PercentOutput, 0);
            } else {
                m_climberMotorRight.set(ControlMode.Velocity, ClimberConstants.kClimberVelocityUp);
            }if (getTopLeftBB()) {
                m_climberMotorLeft.set(ControlMode.PercentOutput, 0);
            } else {
                m_climberMotorLeft.set(ControlMode.Velocity, ClimberConstants.kClimberVelocityUp);
            }    
        }
    }

    public void setClimberVelocityDown() {
        if(enableClimb) {
            
            if (getBottomRightLimit()) {
                m_climberMotorRight.set(ControlMode.PercentOutput, 0);
            } else { 
                m_climberMotorRight.set(ControlMode.Velocity, ClimberConstants.kClimberVelocityDown);
            }
            if (getBottomLeftLimit()) {
                m_climberMotorLeft.set(ControlMode.PercentOutput, 0);
            } else { 
                m_climberMotorLeft.set(ControlMode.Velocity, ClimberConstants.kClimberVelocityDown);
            }
        }
    }

    public void climberInitiateUp() {
        if (enableClimb) {
            if (getTopRightBB()) {
                m_climberMotorRight.set(ControlMode.PercentOutput, 0);
            } else {
                m_climberMotorRight.set(ControlMode.PercentOutput, ClimberConstants.kClimberPowerUp);
            }
            if (getTopLeftBB()) {
                m_climberMotorLeft.set(ControlMode.PercentOutput, 0);
            } else {
                m_climberMotorLeft.set(ControlMode.PercentOutput, ClimberConstants.kClimberPowerUp);
            }
        }
    }

    //TODO tune PID for velocity instaid of percent output in climber up/down
    public void climberUp() {
        if (enableClimb) {
            if (climberExtended) {
                coastClimber();
            }
            if (getTopRightBB()) {
                m_climberMotorRight.set(ControlMode.PercentOutput, 0);
            } else {
                m_climberMotorRight.set(ControlMode.PercentOutput, ClimberConstants.kClimberPowerUp);
            }
            if (getTopLeftBB()) {
                m_climberMotorLeft.set(ControlMode.PercentOutput, 0);
            } else {
                m_climberMotorLeft.set(ControlMode.PercentOutput, ClimberConstants.kClimberPowerUp);
            }
        }
    }

    public void climberDown() {
        if (enableClimb) {
            if (climberExtended) {
                m_climberMotorLeft.setNeutralMode(NeutralMode.Brake);
                m_climberMotorRight.setNeutralMode(NeutralMode.Brake);
            }
            if (getBottomRightLimit()) {
                m_climberMotorRight.set(ControlMode.PercentOutput, 0);
            } else {
                m_climberMotorRight.set(ControlMode.PercentOutput, ClimberConstants.kClimberPowerDown);
            }
            if (getBottomLeftLimit()) {
                m_climberMotorLeft.set(ControlMode.PercentOutput, 0);
            } else {
                m_climberMotorLeft.set(ControlMode.PercentOutput, ClimberConstants.kClimberPowerDown);
            }
        }
    }

    public void coastClimber() {
        m_climberMotorLeft.setNeutralMode(NeutralMode.Coast);
        m_climberMotorRight.setNeutralMode(NeutralMode.Coast);
    }

    public void climberUpSlow() {
        if (getTopRightBB()) {
            m_climberMotorRight.set(ControlMode.PercentOutput, 0);
        } else {
            m_climberMotorRight.set(ControlMode.PercentOutput, ClimberConstants.kClimberSlowUp);
        }
        if (getTopLeftBB()) {
            m_climberMotorLeft.set(ControlMode.PercentOutput, 0);
        } else {
            m_climberMotorLeft.set(ControlMode.PercentOutput, ClimberConstants.kClimberSlowUp);
        }
    }

    public void climberDownSlow() {
        if (getBottomRightLimit()) {
            m_climberMotorRight.set(ControlMode.PercentOutput, 0);
        } else {
            m_climberMotorRight.set(ControlMode.PercentOutput, ClimberConstants.kClimberSlowDown);
        }
        if (getBottomLeftLimit()) {
            m_climberMotorLeft.set(ControlMode.PercentOutput, 0);
        } else {
            m_climberMotorLeft.set(ControlMode.PercentOutput, ClimberConstants.kClimberSlowDown);
        }
    }

    public void climberExtend() {
        if (enableClimb) {
            m_climberSolenoid.set(DoubleSolenoid.Value.kForward);
            climberExtended = true;
        }
    }

    public void climberResetSolenoid() {
        m_climberSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void climberSolenoidOff() {
        m_climberSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    public void stop() {
        m_climberMotorRight.set(ControlMode.PercentOutput, 0);
        m_climberMotorLeft.set(ControlMode.PercentOutput, 0);
    }

    public void initiateClimb(){
        enableClimb = true;
        //reset CAN Status Frame Periods to default values when we want to climb
        m_climberMotorRight.setStatusFramePeriod(1, 10);
        m_climberMotorRight.setStatusFramePeriod(2, 20);
        m_climberMotorLeft.setStatusFramePeriod(1, 10);
        m_climberMotorLeft.setStatusFramePeriod(2, 20);
    }

    public void disableClimb(){
        enableClimb = false;
    }
    
    public boolean getClimbeEnable(){
        return enableClimb;
    }

    public boolean getTopRightBB() {
        return rightTopBreakBeam.get();
    }

    public boolean getTopLeftBB() {
        return leftTopBreakBeam.get();
    }

    public boolean getBottomRightLimit() {
        return rightBottomLimitSwitch.get();
    }

    public boolean getBottomLeftLimit() {
        return leftBottomLimitSwitch.get();
    }

    public double getClimberRightEncoder(){
        return m_climberMotorRight.getSelectedSensorPosition();
    }

    public double getClimberLeftEncoder(){
        return m_climberMotorLeft.getSelectedSensorPosition();
    }

    public void setEncodersZero(){
        m_climberMotorLeft.setSelectedSensorPosition(0);
        m_climberMotorRight.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        topRightBB.setBoolean(getTopRightBB());
        topLeftBB.setBoolean(getTopLeftBB());
        bottomLeftLimit.setBoolean(getBottomLeftLimit());
        bottomRightLimit.setBoolean(getBottomRightLimit());
        climberEnable.setBoolean(getClimbeEnable());
    }

}

