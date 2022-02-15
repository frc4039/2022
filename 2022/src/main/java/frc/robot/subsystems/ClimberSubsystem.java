package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    
    private final TalonFX m_climberMotorRight;

    private final TalonFX m_climberMotorLeft;

    public ClimberSubsystem() {
        m_climberMotorRight = new TalonFX(ClimberConstants.kClimberMotorRightPort);
        m_climberMotorLeft = new TalonFX(ClimberConstants.kClimberMotorLeftPort);
        m_climberMotorRight.configFactoryDefault();
        m_climberMotorLeft.configFactoryDefault();
        m_climberMotorRight.setInverted(ClimberConstants.kClimberMotorRightInversion);
        m_climberMotorLeft.setInverted(ClimberConstants.kClimberMotorLeftInversion);
        m_climberMotorRight.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        m_climberMotorLeft.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        m_climberMotorRight.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        m_climberMotorLeft.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
        m_climberMotorRight.setNeutralMode(NeutralMode.Brake);
        m_climberMotorLeft.setNeutralMode(NeutralMode.Brake);

        m_climberMotorLeft.configReverseSoftLimitThreshold(0);
        m_climberMotorLeft.configForwardSoftLimitThreshold(ClimberConstants.kFullyClimbedTicks);
        m_climberMotorRight.configReverseSoftLimitThreshold(0);
        m_climberMotorRight.configForwardSoftLimitThreshold(ClimberConstants.kFullyClimbedTicks);
        
        m_climberMotorLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        m_climberMotorRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        m_climberMotorLeft.getSensorCollection().setIntegratedSensorPosition(0, ClimberConstants.kTimeoutMs);
        m_climberMotorRight.getSensorCollection().setIntegratedSensorPosition(0, ClimberConstants.kTimeoutMs);

        /*
        m_climberMotorLeft.configReverseSoftLimitEnable(true);
        m_climberMotorLeft.configForwardSoftLimitEnable(true);
        m_climberMotorRight.configReverseSoftLimitEnable(true);
        m_climberMotorRight.configForwardSoftLimitEnable(true);
        */
        
    }
    
    public void climberUp() {
        m_climberMotorRight.set(ControlMode.PercentOutput, ClimberConstants.kClimberPowerUp);
        m_climberMotorLeft.set(ControlMode.PercentOutput, ClimberConstants.kClimberPowerUp);
    }

    public void climberDown() {
        m_climberMotorRight.set(ControlMode.PercentOutput, ClimberConstants.kClimberPowerDown);
        m_climberMotorLeft.set(ControlMode.PercentOutput, ClimberConstants.kClimberPowerDown);
    }

    public void climberUpSlow() {
        m_climberMotorRight.set(ControlMode.PercentOutput, ClimberConstants.kClimberSlowUp);
        m_climberMotorLeft.set(ControlMode.PercentOutput, ClimberConstants.kClimberSlowUp);
    }

    public void climberDownSlow() {
        m_climberMotorRight.set(ControlMode.PercentOutput, ClimberConstants.kClimberSlowDown);
        m_climberMotorLeft.set(ControlMode.PercentOutput, ClimberConstants.kClimberSlowDown);
    }

    public void stop() {
        m_climberMotorRight.set(ControlMode.PercentOutput, 0);
        m_climberMotorLeft.set(ControlMode.PercentOutput, 0);
    }

    public boolean getTopRightLimit() {
        return m_climberMotorRight.getSensorCollection().isFwdLimitSwitchClosed() == 1;
    }

    public boolean getTopLeftLimit() {
        return m_climberMotorLeft.getSensorCollection().isFwdLimitSwitchClosed() == 1;
    
    }

    public boolean getBottomRightLimit() {
        return m_climberMotorRight.getSensorCollection().isRevLimitSwitchClosed() == 1;
    }

    public boolean getBottomLeftLimit() {
        return m_climberMotorLeft.getSensorCollection().isRevLimitSwitchClosed() == 1;
    }

    public void overrideLimitSwitches() {
        m_climberMotorRight.overrideLimitSwitchesEnable(true);
        m_climberMotorLeft.overrideLimitSwitchesEnable(true);
    }

    public double getClimberRightEncoder(){
        return m_climberMotorRight.getSelectedSensorPosition();
    }

    public double getClimberLeftEncoder(){
        return m_climberMotorLeft.getSelectedSensorPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Right Encoder", getClimberRightEncoder());
        SmartDashboard.putNumber("Climber Left Encoder", getClimberLeftEncoder());
    }

}

