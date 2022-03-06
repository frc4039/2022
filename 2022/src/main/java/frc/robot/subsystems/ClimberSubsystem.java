package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    
    private final TalonFX m_climberMotorRight;

    private final TalonFX m_climberMotorLeft;

    private boolean enableClimb = false;

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

        leftBottomLimitSwitch = new DigitalInput(ClimberConstants.kLeftBottomLimitSwitchPort);
        rightBottomLimitSwitch = new DigitalInput(ClimberConstants.kRightBottomLimitSwitchPort);
        leftTopBreakBeam = new DigitalInput(ClimberConstants.kLeftTopBreakBeamPort);
        rightTopBreakBeam = new DigitalInput(ClimberConstants.kRightTopBreakBeamPort);

        ShuffleboardTab tab = Shuffleboard.getTab("Driver Readout");
        
        topLeftBB = tab.add("Top Left B B", false)
                .withPosition(2, 1)
                .withSize(1, 1)
                .getEntry();
        topRightBB = tab.add("Top Right B B", false)
                .withPosition(3, 1)
                .withSize(1, 1)
                .getEntry();
        bottomLeftLimit = tab.add("Bottom Left Limit", false)
                .withPosition(4, 1)
                .withSize(1, 1)
                .getEntry();
        bottomRightLimit = tab.add("Bottom Right Limit", false)
                .withPosition(5, 1)
                .withSize(1, 1)
                .getEntry();
        climberEnable = tab.add("Climber Enabled", false)
                .withPosition(6, 1)
                .withSize(1, 1)
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

    public void stop() {
        m_climberMotorRight.set(ControlMode.PercentOutput, 0);
        m_climberMotorLeft.set(ControlMode.PercentOutput, 0);
    }

    public void initiateClimb(){
        enableClimb = true;
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
        /*
        SmartDashboard.putNumber("Climber Right Encoder", getClimberRightEncoder());
        SmartDashboard.putNumber("Climber Left Encoder", getClimberLeftEncoder());
        SmartDashboard.putBoolean("Climber Top Right BB", getTopRightBB());
        SmartDashboard.putBoolean("Climber Top Left BB", getTopLeftBB());
        SmartDashboard.putBoolean("Climber Bottom Right LS", getBottomRightLimit());
        SmartDashboard.putBoolean("Climber Bottom Left LS", getBottomLeftLimit());
        SmartDashboard.putBoolean("Climber Enable", getClimbeEnable());
        */
        topRightBB.setBoolean(getTopRightBB());
        topLeftBB.setBoolean(getTopLeftBB());
        bottomLeftLimit.setBoolean(getBottomLeftLimit());
        bottomRightLimit.setBoolean(getBottomRightLimit());
        climberEnable.setBoolean(getClimbeEnable());
    }

}

