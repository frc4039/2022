package frc.robot.common.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.common.math.Rotation2;

public class DriverReadout {

    public DriverReadout(RobotContainer container) {
        ShuffleboardTab tab = Shuffleboard.getTab("Driver Readout");

        tab.add("Autonomous Mode", container.getAutonomousChooser().getAutonomousModeChooser())
                .withSize(2, 1)
                .withPosition(1, 0);
        tab.add("Zero Gyroscope", new ZeroGyroscope(container.getDrivetrainSubsystem()))
                .withSize(1, 1)
                .withPosition(6, 1);
    }

    private static class ZeroGyroscope extends CommandBase {
        private final DrivetrainSubsystem drivetrain;

        public ZeroGyroscope(DrivetrainSubsystem drivetrain) {
            this.drivetrain = drivetrain;

            setName("Zero Gyroscope");
        }

        @Override
        public void initialize() {
            drivetrain.resetGyroAngle(Rotation2.ZERO);
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
}