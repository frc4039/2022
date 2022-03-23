package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RumbleCommand extends CommandBase {
    private final RobotContainer m_RobotContainer;
    

    public RumbleCommand(RobotContainer robotContainer) {
        m_RobotContainer = robotContainer;


        addRequirements();
    }

    @Override
    public void initialize() {
        m_RobotContainer.getDriverController().getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
        m_RobotContainer.getDriverController().getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
        m_RobotContainer.getOperatorController().getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
        m_RobotContainer.getOperatorController().getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
        System.out.println("start rumble");
    }

    @Override
    public void execute() {
        System.out.println("rumblerumble");
    }

    @Override
    public void end(boolean interrupted) {
        m_RobotContainer.getDriverController().getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
        m_RobotContainer.getDriverController().getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        m_RobotContainer.getOperatorController().getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
        m_RobotContainer.getOperatorController().getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        System.out.println("stop rumble");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
