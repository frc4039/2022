// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.common.autonomous.AutonomousChooser;
import frc.robot.common.autonomous.AutonomousTrajectories;
import frc.robot.common.util.DriverReadout;
import frc.robot.common.math.Rotation2;
import frc.robot.common.math.Vector2;
import frc.robot.common.swervelib.DriveController;
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
  //TODO Clean up m_ for subsystems
  private final XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
  private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();


  private AutonomousTrajectories autonomousTrajectories;
  private AutonomousChooser autonomousChooser;

  private final DriverReadout driverReadout;

  private final double RPM = 2800;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings

    try {
      autonomousTrajectories = new AutonomousTrajectories(DrivetrainSubsystem.TRAJECTORY_CONSTRAINTS);
    } catch (IOException e) {
      e.printStackTrace();
    }
    autonomousChooser = new AutonomousChooser(autonomousTrajectories);

    driverController.getLeftYAxis().setInverted(true);
    driverController.getLeftXAxis().setInverted(true);
    CommandScheduler.getInstance().registerSubsystem(shooterSubsystem);
    CommandScheduler.getInstance().registerSubsystem(feederSubsystem);
    CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
    CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
    CommandScheduler.getInstance().registerSubsystem(m_climberSubsystem);
    CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));


    driverReadout = new DriverReadout(this);

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

    
    driverController.getRightTriggerAxis().getButton(0.1).whenHeld(
        new ShootCommand(shooterSubsystem, feederSubsystem)
    );

    //TODO Change back to this once intake pneumatics are ready
    // driverController.getRightTriggerAxis().getButton(0.1).whenHeld(
    //   new ShootCommand(shooterSubsystem, feederSubsystem)
    // );

    //D-pad up increases shooter RPM by 25
    driverController.getDPadButton(Direction.UP).whenPressed(
      new ChangeShooterRPM(shooterSubsystem, true)
    );

    //D-pad down decreases shooter RPM by 25
    driverController.getDPadButton(Direction.DOWN).whenPressed(
      new ChangeShooterRPM(shooterSubsystem, false)
    );

    //D-pad right decreases shooter RPM by 25
    driverController.getDPadButton(Direction.RIGHT).whenPressed(
      new ChangePreShooterRPM(shooterSubsystem, false)
    );

    //D-pad left increases shooter RPM by 25
    driverController.getDPadButton(Direction.LEFT).whenPressed(
      new ChangePreShooterRPM(shooterSubsystem, true)
    );

    //A button shoots

    /*

    driverController.getAButton().whenPressed(
      new ShootCommand(shooterSubsystem, feederSubsystem)
    );
    */

    driverController.getLeftTriggerAxis().getButton(0.1).whenHeld(
      new ParallelCommandGroup(
        new IntakeCommand(intakeSubsystem),
        new FeederCommand(feederSubsystem, ShooterConstants.kSlowFeederPercent)
      )
    );

    operatorController.getLeftTriggerAxis().getButton(0.5).whenHeld(
      new ClimberDownCommand(m_climberSubsystem)
    );

    operatorController.getLeftBumperButton().whenHeld(
      new ClimberDownSlowCommand(m_climberSubsystem)
    );
    
    operatorController.getRightTriggerAxis().getButton(0.5).whenHeld(
      new ClimberUpCommand(m_climberSubsystem)
    );

    operatorController.getRightBumperButton().whenHeld(
      new ClimberUpSlowCommand(m_climberSubsystem)
    );

    operatorController.getAButton().whenPressed(
      new FeederCommand(feederSubsystem, ShooterConstants.kFeederPercent)
    );

    operatorController.getBButton().whenPressed(
      new FeederCommand(feederSubsystem, -ShooterConstants.kFeederPercent)
    );

    operatorController.getXButton().whenPressed(
      new InstantCommand(feederSubsystem::feederSlowBackward, feederSubsystem)
    );

    operatorController.getXButton().whenReleased(
      new InstantCommand(feederSubsystem::stop, feederSubsystem)
    );

    operatorController.getAButton().whenReleased(
      new InstantCommand(feederSubsystem::stop, feederSubsystem)
    );

    operatorController.getBButton().whenReleased(
      new InstantCommand(feederSubsystem::stop, feederSubsystem)
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
