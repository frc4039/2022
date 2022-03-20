package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.GenericHID;

public class FeederManagementCommand extends CommandBase {

	private final FeederSubsystem m_feeder;
	private final RobotContainer m_robotContainer;

	private Debouncer rumbleDebounce = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

	public FeederManagementCommand(FeederSubsystem feeder, RobotContainer robotContainer) {
		this.m_feeder = feeder;
		m_robotContainer = robotContainer;

		addRequirements(m_feeder);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (!(m_feeder.getBothBallBreakBeams())){
			if (m_feeder.getBreakBeamIntake()){
				m_feeder.runFeeder(FeederConstants.kFeederFeedPercent);
			} else {
				m_feeder.stop();
			}
		} else if (/*m_feeder.getBothBallBreakBeams() &&*/ m_feeder.getBreakBeamPreShooter()){
			m_feeder.runFeeder(-FeederConstants.kFeederFeedPercent);
		} else {
			m_feeder.stop();
		}

		/*
		if(rumbleDebounce.calculate(m_feeder.getBreakBeamIntake())){
			m_robotContainer.getDriverController().getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
			m_robotContainer.getDriverController().getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
			m_robotContainer.getOperatorController().getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
			m_robotContainer.getOperatorController().getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
		} else {
			m_robotContainer.getDriverController().getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
			m_robotContainer.getDriverController().getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
			m_robotContainer.getOperatorController().getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
			m_robotContainer.getOperatorController().getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
		}
		*/
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
