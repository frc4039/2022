// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.common.autonomous.AutonomousChooser;
import frc.robot.common.autonomous.AutonomousTrajectories;
import frc.robot.common.util.DriverReadout;
import frc.robot.common.math.Rotation2;
import frc.robot.common.math.Vector2;
import frc.robot.common.input.Axis;
import frc.robot.common.input.DPadButton;
import frc.robot.common.input.XboxController;
import frc.robot.common.input.DPadButton.Direction;
import frc.robot.Constants.*;

import java.io.IOException;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
  private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private AutonomousTrajectories autonomousTrajectories;
  private AutonomousChooser autonomousChooser;

  private final DriverReadout driverReadout;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings

    try {
      autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS);
    } catch (IOException e) {
      e.printStackTrace();
    }
    autonomousChooser = new AutonomousChooser(autonomousTrajectories);

    driverController.getLeftXAxis().setInverted(true);
    driverController.getRightXAxis().setInverted(true);

    CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
    CommandScheduler.getInstance().registerSubsystem(shooterSubsystem);

    CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));

    driverReadout = new DriverReadout(this);

    SmartDashboard.putNumber("Shooter RPM Setpoint", 0.0);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverController.getBackButton().whenPressed(
      () -> drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO)
    );

    //B button stops shooter
    driverController.getBButton().whenPressed(
      new InstantCommand(shooterSubsystem::stop, shooterSubsystem)
    );

    //D-pad up sets shooter to 3000rpm
    driverController.getDPadButton(Direction.UP).whenPressed(
      new ShootCommand(shooterSubsystem, 3000)
    );

    //D-pad up-right sets shooter to 3100rpm
    driverController.getDPadButton(Direction.UPRIGHT).whenPressed(
      new ShootCommand(shooterSubsystem, 3100)
    );

    //D-pad right sets shooter to 3200rpm
    driverController.getDPadButton(Direction.RIGHT).whenPressed(
      new ShootCommand(shooterSubsystem, 3200)
    );

    //D-pad down-right sets shooter to 3300rpm
    driverController.getDPadButton(Direction.DOWNRIGHT).whenPressed(
      new ShootCommand(shooterSubsystem, 3300)
    );

    //D-pad down sets shooter to 3400rpm
    driverController.getDPadButton(Direction.DOWN).whenPressed(
      new ShootCommand(shooterSubsystem, 3400)
    );

    //D-pad down-left sets shooter to 3500rpm
    driverController.getDPadButton(Direction.DOWNLEFT).whenPressed(
      new ShootCommand(shooterSubsystem, 3500)
    );

    //D-pad left sets shooter to 3200rpm
    driverController.getDPadButton(Direction.LEFT).whenPressed(
      new ShootCommand(shooterSubsystem, 3600)
    );

    //D-pad left-up sets shooter to 3700rpm
    driverController.getDPadButton(Direction.UPLEFT).whenPressed(
      new ShootCommand(shooterSubsystem, 3700)
    );

    //A button sets shooter to Driver Station RPM input
    driverController.getAButton().whenPressed(
      new ShootCommand(shooterSubsystem, SmartDashboard.getNumber("Shooter RPM Setpoint", 0.0))
    );

    driverController.getRightBumperButton().whenHeld(
      new InstantCommand(shooterSubsystem::shooterSlowForward, shooterSubsystem)
    );

    driverController.getLeftBumperButton().whenHeld(
      new InstantCommand(shooterSubsystem::shooterSlowBackward, shooterSubsystem)
    );

  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand(this);
  }

  private Axis getDriveForwardAxis() {
    return driverController.getLeftYAxis();
  }

  private Axis getDriveStrafeAxis() {
    return driverController.getLeftXAxis();
  }

  private Axis getDriveRotationAxis() {
    return driverController.getRightXAxis();
  }

  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrainSubsystem;
  }

  public XboxController getDriverController() {
    return driverController;
  }

  public XboxController getOperatorController() {
    return operatorController;
  }

  public AutonomousChooser getAutonomousChooser() {
    return autonomousChooser;
  }

  public void PrintAllValues(){
    shooterSubsystem.printShooterValues();
  }
}
