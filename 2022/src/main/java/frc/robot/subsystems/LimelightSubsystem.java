package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    
    private NetworkTable table;
    private static final double TARGET_HEIGHT = 103; //8 feet 7 inches 
    // TODO: Update limelight height
    private static final double LIMELIGHT_HEIGHT = 25.25;
    private static final double LIMELIGHT_ANGLE = 31.6;

    private final NetworkTableEntry limeLightTableEntry;
    private final NetworkTableEntry limeLightTargetTableEntry;

    public LimelightSubsystem() {
        
        table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("pipeline").setNumber(0);
        
        ShuffleboardTab tab = Shuffleboard.getTab("Driver Readout");

        limeLightTableEntry = tab.add("Limelight Working", false)
                .withPosition(4, 2)
                .withSize(1, 1)
                .getEntry();
        
        limeLightTargetTableEntry = tab.add("Limelight Target", false)
                .withPosition(3, 2)
                .withSize(1, 1)
                .getEntry();
    }

    public double getHorzAngleToGoal() {
        return table.getEntry("tx").getDouble(0);
    }

    public double getVertAngleToGoal() {
        return table.getEntry("ty").getDouble(0);
    }

    public boolean getValidTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public double getTargetArea() {
        return table.getEntry("ta").getDouble(0);
    }

    public double getLatency() {
        return table.getEntry("tl").getDouble(0);
    }

    public double getDistanceToTarget() {
        return (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan((LIMELIGHT_ANGLE + getVertAngleToGoal()) * (Math.PI / 180)) + 26;
    }

    public void turnLEDOn() {
        table.getEntry("ledMode").setDouble(0);
    }

    public void turnLEDOff() {
        table.getEntry("ledMode").setDouble(1);
    }

    public void setPipelineZero() {
        table.getEntry("pipeline").setNumber(1);
    }

    public void setPicInPicCam() {
        table.getEntry("stream").setNumber(2);
    }

    public void setPicInPicLL() {
        table.getEntry("stream").setNumber(1);
    }

    public void setCamModeCam() {
        table.getEntry("camMode").setNumber(1);
    }

    public void setCamModeLL() {
        table.getEntry("camMode").setNumber(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LL Latency", getLatency());
        SmartDashboard.putNumber("Distance To Goal", getDistanceToTarget());
        
        limeLightTableEntry.setBoolean(getLatency() != 0);
        limeLightTargetTableEntry.setBoolean(getValidTarget());
    }
}