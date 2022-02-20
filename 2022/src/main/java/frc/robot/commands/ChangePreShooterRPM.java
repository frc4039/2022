package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ChangePreShooterRPM extends CommandBase {
    private final ShooterSubsystem m_shooter;
    private final boolean m_increase;

    public ChangePreShooterRPM(ShooterSubsystem shooter, boolean increase) {
        m_shooter = shooter;
        m_increase = increase;

        addRequirements(m_shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_increase) {
            m_shooter.PreShooterRPM += 25;
        } else {
            m_shooter.PreShooterRPM -= 25;
        }
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("PreShooter RPM, SetPoint", m_shooter.PreShooterRPM);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
