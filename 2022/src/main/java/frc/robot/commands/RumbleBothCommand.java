package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RumbleBothCommand extends CommandBase {
    private final RobotContainer m_robotContainer;

    public RumbleBothCommand(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_robotContainer.getDriverRumble().setRumble(RumbleType.kLeftRumble, 1);
        m_robotContainer.getDriverRumble().setRumble(RumbleType.kRightRumble, 1);
        
        m_robotContainer.getOperatorRumble().setRumble(RumbleType.kLeftRumble, 1);
        m_robotContainer.getOperatorRumble().setRumble(RumbleType.kRightRumble, 1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_robotContainer.getDriverRumble().setRumble(RumbleType.kLeftRumble, 0);
        m_robotContainer.getDriverRumble().setRumble(RumbleType.kRightRumble, 0);
        
        m_robotContainer.getOperatorRumble().setRumble(RumbleType.kLeftRumble, 0);
        m_robotContainer.getOperatorRumble().setRumble(RumbleType.kRightRumble, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
