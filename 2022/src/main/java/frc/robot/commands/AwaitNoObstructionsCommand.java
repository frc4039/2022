package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

public class AwaitNoObstructionsCommand extends CommandBase {

	private final FeederSubsystem m_feeder;

	public AwaitNoObstructionsCommand(FeederSubsystem feeder) {
		this.m_feeder = feeder;
	}

	@Override
	public boolean isFinished() {
		return m_feeder.getNoObstructions();
	}
}
