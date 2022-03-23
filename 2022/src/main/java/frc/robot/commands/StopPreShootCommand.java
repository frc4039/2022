package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PreShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem;

public class StopPreShootCommand extends CommandBase {

    private final PreShooterSubsystem m_preShooter;
    private final ShooterSubsystem m_shooter;
    private final FeederSubsystem m_feeder;
    

    public StopPreShootCommand(PreShooterSubsystem preShooter, ShooterSubsystem shooter, FeederSubsystem feeder) {
        this.m_preShooter = preShooter;
        this.m_shooter = shooter;
        this.m_feeder = feeder;

        addRequirements(m_preShooter, m_shooter, m_feeder);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_feeder.stop();
        m_preShooter.stop();
        m_shooter.stop();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

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