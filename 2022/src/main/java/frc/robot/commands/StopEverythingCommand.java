package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StopEverythingCommand extends CommandBase {
    private final ClimberSubsystem m_climberSubsystem;
    private final DrivetrainSubsystem m_driveTrainSubsystem;
    private final FeederSubsystem m_feederSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final PreShooterSubsystem m_preShooterSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    

    public StopEverythingCommand(ClimberSubsystem climberSubsystem, DrivetrainSubsystem drivetrainSubsystem, FeederSubsystem feederSubsystem, IntakeSubsystem intakeSubsystem, PreShooterSubsystem preShooterSubsystem, ShooterSubsystem shooterSubsystem) {
        m_climberSubsystem = climberSubsystem;
        m_driveTrainSubsystem = drivetrainSubsystem;
        m_feederSubsystem = feederSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_preShooterSubsystem = preShooterSubsystem;
        m_shooterSubsystem = shooterSubsystem;

        addRequirements(m_climberSubsystem, m_driveTrainSubsystem, m_feederSubsystem, m_intakeSubsystem, m_preShooterSubsystem, m_shooterSubsystem);
    }

    @Override
    public void initialize() {
        m_climberSubsystem.stop();
        m_feederSubsystem.stop();
        m_intakeSubsystem.stop();
        m_preShooterSubsystem.stop();
        m_shooterSubsystem.stop();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
