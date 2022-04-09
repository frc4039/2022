package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.common.input.Axis;

public class AimAndEjectSecondBallCommand extends ParallelCommandGroup{

    public AimAndEjectSecondBallCommand(DrivetrainSubsystem drivetrain, Axis forward, Axis strafe, LimelightSubsystem limelightSubsystem, boolean updateOdometry, ShooterSubsystem shooter, PreShooterSubsystem preShooter, FeederSubsystem feeder) {
        addCommands(
            new RotateToLimelightCommand(drivetrain, forward, strafe, limelightSubsystem, updateOdometry),
            new EjectSecondBallCommand(shooter, preShooter, feeder, limelightSubsystem, drivetrain)
        );
    }
    
}
