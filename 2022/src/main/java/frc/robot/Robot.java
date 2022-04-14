// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.StopEverythingCommand;
import frc.robot.common.UpdateManager;
import frc.robot.common.math.RigidTransform2;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static Robot instance = null;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer = new RobotContainer();

  private Compressor pcmCompressor = new Compressor(Constants.kPCMCANID, PneumaticsModuleType.CTREPCM);

  DoubleLogEntry logPDPVoltage;
  double oldPDPVoltage;
  DoubleLogEntry logPDPTotalCurrent;
  double oldPDPTotalCurrent;
  BooleanLogEntry logPDPBrownout;
  boolean oldPDPBrownout;
  DoubleLogEntry[] logPDPCurrent = new DoubleLogEntry[24];
  double[] oldPDPCurrent = new double[24];

  private UpdateManager updateManager = new UpdateManager(
      m_robotContainer.getDrivetrainSubsystem()
  );

  public Robot() {
    instance = this;
  }

  public static Robot getInstance(){
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    
    
    
    //TODO: removed based on 2910 code, probably have to add all subsystems to UpdateManager
    //m_robotContainer = new RobotContainer();

    DataLogManager.start();

    DataLog log = DataLogManager.getLog();
    logPDPVoltage = new DoubleLogEntry(log, "/pdp/voltage");
    oldPDPVoltage = m_robotContainer.getPowerDistributionSubsystem().getVoltage();
    logPDPTotalCurrent = new DoubleLogEntry(log, "/pdp/current/total");
    oldPDPTotalCurrent = m_robotContainer.getPowerDistributionSubsystem().getTotalCurrent();
    logPDPBrownout = new BooleanLogEntry(log, "/pdp/fault/brownout");
    oldPDPBrownout = m_robotContainer.getPowerDistributionSubsystem().getBrownout();
    for (int i = 0; i < logPDPCurrent.length; i++) {
      logPDPCurrent[i] = new DoubleLogEntry(log, "/pdp/current/" + i);
      oldPDPCurrent[i] = m_robotContainer.getPowerDistributionSubsystem().getChannelCurrent(i);
    }
    
    updateManager.startLoop(5.0e-3);
    pcmCompressor.enableDigital();
    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    if (m_robotContainer.getPowerDistributionSubsystem().getVoltage() != oldPDPVoltage) {
      logPDPVoltage.append(m_robotContainer.getPowerDistributionSubsystem().getVoltage());
      oldPDPVoltage = m_robotContainer.getPowerDistributionSubsystem().getVoltage();
    }
    if (m_robotContainer.getPowerDistributionSubsystem().getTotalCurrent() != oldPDPTotalCurrent) {
      logPDPTotalCurrent.append(m_robotContainer.getPowerDistributionSubsystem().getTotalCurrent());
      oldPDPTotalCurrent = m_robotContainer.getPowerDistributionSubsystem().getTotalCurrent();
    }
    if (m_robotContainer.getPowerDistributionSubsystem().getBrownout() != oldPDPBrownout) {
      logPDPBrownout.append(m_robotContainer.getPowerDistributionSubsystem().getBrownout());
      oldPDPBrownout = m_robotContainer.getPowerDistributionSubsystem().getBrownout();
    }
    for (int i = 0; i < logPDPCurrent.length; i++) {
      if (m_robotContainer.getPowerDistributionSubsystem().getChannelCurrent(i) != oldPDPCurrent[i]) {
        logPDPCurrent[i].append(m_robotContainer.getPowerDistributionSubsystem().getChannelCurrent(i));
        oldPDPCurrent[i] = m_robotContainer.getPowerDistributionSubsystem().getChannelCurrent(i);
      }
    }

    CommandScheduler.getInstance().run();
    m_robotContainer.PrintAllValues();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.disableClimber();
    m_robotContainer.stopEverything();

    }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.getDrivetrainSubsystem().resetPose(RigidTransform2.ZERO);
    m_robotContainer.getAutonomousCommand().schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
