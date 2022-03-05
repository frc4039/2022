package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

public class FeederManagementCommand extends CommandBase {

	private final FeederSubsystem m_feeder;

	public FeederManagementCommand(FeederSubsystem feeder) {
		this.m_feeder = feeder;

		addRequirements(m_feeder);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
        if (m_feeder.getBothBreakBeams()){
            //m_feeder.runFeederReverse(FeederConstants.kFeederFeedPercent);
			m_feeder.stop();
        } else if (m_feeder.getBreakBeamIntake()){
            m_feeder.runFeeder(FeederConstants.kFeederFeedPercent);
        } else if (m_feeder.getBreakBeamPreShooter()){
            m_feeder.runFeederReverse(FeederConstants.kFeederFeedPercent);
        } else {
            m_feeder.stop();
        }
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
