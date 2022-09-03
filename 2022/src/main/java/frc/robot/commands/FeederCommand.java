package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

public class FeederCommand extends CommandBase {

	private final FeederSubsystem m_feeder;
	private final double m_speed;

	public FeederCommand(FeederSubsystem feeder, double speed) {
		this.m_feeder = feeder;
		this.m_speed = speed;

		addRequirements(m_feeder);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_feeder.runFeeder(m_speed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_feeder.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
