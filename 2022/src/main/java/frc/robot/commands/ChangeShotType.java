package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ChangeShotType extends CommandBase {
    private final ShooterSubsystem m_shooterSubsystem;
    private final PreShooterSubsystem m_preShooterSubsystem;
    private final int m_type;

    public ChangeShotType(ShooterSubsystem shooter, PreShooterSubsystem preShooter, int type) {
        m_shooterSubsystem = shooter;
        m_preShooterSubsystem = preShooter;
        m_type = type;

        addRequirements(m_shooterSubsystem, m_preShooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shooterSubsystem.type = m_type;
        m_preShooterSubsystem.type = m_type;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
