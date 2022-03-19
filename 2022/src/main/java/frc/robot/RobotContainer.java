// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.common.autonomous.AutonomousChooser;
import frc.robot.common.autonomous.AutonomousTrajectories;
import frc.robot.common.util.DriverReadout;
import frc.robot.common.math.Rotation2;
import frc.robot.common.input.Axis;
import frc.robot.common.input.XboxController2;
import frc.robot.common.input.DPadButton.Direction;

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
  private final XboxController2 driverController = new XboxController2(Constants.DRIVER_CONTROLLER_PORT);
  private final XboxController2 operatorController = new XboxController2(Constants.OPERATOR_CONTROLLER_PORT);
  // private final XboxController driverRumble = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
  // private final XboxController operatorRumble = new XboxController(Constants.OPERATOR_CONTROLLER_PORT);

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final PreShooterSubsystem preShooterSubsystem = new PreShooterSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();


  private AutonomousTrajectories autonomousTrajectories;
  private AutonomousChooser autonomousChooser;

  private final DriverReadout driverReadout;

  private final Trigger intakeBB = new Trigger(feederSubsystem::getBreakBeamIntake);

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
    driverController.getRightXAxis().setInverted(true);
    CommandScheduler.getInstance().registerSubsystem(shooterSubsystem);
    CommandScheduler.getInstance().registerSubsystem(preShooterSubsystem);
    CommandScheduler.getInstance().registerSubsystem(feederSubsystem);
    CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
    CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
    CommandScheduler.getInstance().registerSubsystem(m_climberSubsystem);
    CommandScheduler.getInstance().registerSubsystem(limelightSubsystem);
    CommandScheduler.getInstance().setDefaultCommand(drivetrainSubsystem, new DriveCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), getDriveRotationAxis()));
    CommandScheduler.getInstance().setDefaultCommand(feederSubsystem, new FeederManagementCommand(feederSubsystem));

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
    //Driver back button resets gyro
    driverController.getBackButton().whenPressed(
      () -> drivetrainSubsystem.resetGyroAngle(Rotation2.ZERO)
    );

    //Driver Right Trigger shoots
    driverController.getRightTriggerAxis().getButton(0.1).whenHeld(
        new ShootCommand(shooterSubsystem, preShooterSubsystem, feederSubsystem, limelightSubsystem)
    );

    driverController.getYButton().whenHeld(
      new DriveWithSetRotationCommand(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), 0)
    );

    driverController.getAButton().whenHeld(
      new SequentialCommandGroup(
        new ShooterHoodRetract(shooterSubsystem),
        new ChangeShotType(shooterSubsystem, preShooterSubsystem, "limelight"),
        new InstantCommand(limelightSubsystem::turnLEDOn, limelightSubsystem),
        new RotateToLimelight(drivetrainSubsystem, getDriveForwardAxis(), getDriveStrafeAxis(), limelightSubsystem)
      )
    );
    
    // intakeBB.whenActive(
    //   new RumbleBothCommand(this).withTimeout(2.0)
    // );
  
    //Operator A button intakes balls
    operatorController.getAButton().whenHeld(
      new IntakeCommand(intakeSubsystem)
    );

    //Operator Y buttom spools up shooter
    /*
    operatorController.getYButton().whenPressed(
      new PreShootCommand(preShooterSubsystem, shooterSubsystem, feederSubsystem)
    );
    */

    operatorController.getBButton().whenHeld(
      new EjectOutOfIntake(feederSubsystem, intakeSubsystem)
    );
      
    operatorController.getXButton().whenHeld(
      new EjectOutOfShooter(shooterSubsystem, preShooterSubsystem, feederSubsystem)
    );

    operatorController.getBackButton().and(operatorController.getStartButton()).whenActive(
      new SequentialCommandGroup(
        new ClimberEncoderZeroCommand(m_climberSubsystem),
        new ClimberPreClimbCommand(m_climberSubsystem)
      )
    );

    operatorController.getLeftTriggerAxis().getButton(0.5).whenHeld(
      new ClimberDownCommand(m_climberSubsystem)
    );
    
    operatorController.getRightTriggerAxis().getButton(0.5).whenHeld(
      new ClimberUpCommand(m_climberSubsystem)
    );

    operatorController.getLeftBumperButton().and(operatorController.getBackButton()).whileActiveOnce(
      new ClimberDownSlowCommand(m_climberSubsystem)
    );

    operatorController.getRightBumperButton().and(operatorController.getBackButton()).whileActiveOnce(
      new ClimberUpSlowCommand(m_climberSubsystem)
    );

    operatorController.getLeftBumperButton().and(operatorController.getRightBumperButton()).whenActive(
      new SequentialCommandGroup(
        new ClimberExtendCommand(m_climberSubsystem),
        new WaitCommand(1),
        new InstantCommand(m_climberSubsystem::climberSolenoidOff, m_climberSubsystem)
      )
    );

    operatorController.getDPadButton(Direction.LEFT).whenPressed(
      new SequentialCommandGroup(
        new ShooterHoodRetract(shooterSubsystem),
        new ChangeShotType(shooterSubsystem, preShooterSubsystem, "limelight"),
        new InstantCommand(limelightSubsystem::turnLEDOn, limelightSubsystem)
      )
    );

    operatorController.getDPadButton(Direction.UP).whenPressed(
      new SequentialCommandGroup(
        new ShooterHoodExtend(shooterSubsystem),
        new ChangeShotType(shooterSubsystem, preShooterSubsystem, "high"),
        new InstantCommand(limelightSubsystem::turnLEDOff, limelightSubsystem)
      )
    );

    operatorController.getDPadButton(Direction.DOWN).whenPressed(
      new SequentialCommandGroup(
        new ShooterHoodRetract(shooterSubsystem),
        new ChangeShotType(shooterSubsystem, preShooterSubsystem, "low"),
        new InstantCommand(limelightSubsystem::turnLEDOff, limelightSubsystem)
      )
    );

  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand(this);
  }

  public Axis getDriveForwardAxis() {
    return driverController.getLeftYAxis();
  }

  public Axis getDriveStrafeAxis() {
    return driverController.getLeftXAxis();
  }

  private Axis getDriveRotationAxis() {
    return driverController.getRightXAxis();
  }

  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrainSubsystem;
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return intakeSubsystem;
  }

  public ShooterSubsystem getShooterSubsystem() {
    return shooterSubsystem;
  }

  public PreShooterSubsystem getPreShooterSubsystem() {
    return preShooterSubsystem;
  }

  public LimelightSubsystem getLimelightSubsystem() {
    return limelightSubsystem;
  }

  public FeederSubsystem getFeederSubsystem() {
    return feederSubsystem;
  }

  public XboxController2 getDriverController() {
    return driverController;
  }

  public XboxController2 getOperatorController() {
    return operatorController;
  }

  // public XboxController getDriverRumble() {
  //   return driverRumble;
  // }

  // public XboxController getOperatorRumble() {
  //   return operatorRumble;
  // }

  public AutonomousChooser getAutonomousChooser() {
    return autonomousChooser;
  }

  public void disableClimber(){
    m_climberSubsystem.disableClimb();
  }

  public void stopEverything(){
    new StopEverythingCommand(m_climberSubsystem, drivetrainSubsystem, feederSubsystem, intakeSubsystem, preShooterSubsystem, shooterSubsystem);
  }

  public void PrintAllValues(){
    shooterSubsystem.printShooterValues();
  }
}