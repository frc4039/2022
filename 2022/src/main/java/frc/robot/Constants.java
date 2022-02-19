// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int kPCMCANID = 25;

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;


    public static final double kDeadband = 0.075;
    public static final double kRotationScale = 0.5;

    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 1;
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 3;
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 5;
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 7;
    
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 2;
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 4;
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 8;
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 6;

    public static final int DRIVETRAIN_FRONT_LEFT_ENCODER_PORT = 11;
    public static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_PORT = 12;
    public static final int DRIVETRAIN_BACK_LEFT_ENCODER_PORT = 13;
    public static final int DRIVETRAIN_BACK_RIGHT_ENCODER_PORT = 14;

    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(22.85);
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(31.99);
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(4.83);
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(345.71);


    public static final class ClimberConstants{
        public static final TalonFXInvertType kClimberMotorRightInversion = TalonFXInvertType.Clockwise;
        public static final TalonFXInvertType kClimberMotorLeftInversion = TalonFXInvertType.CounterClockwise;
        public static final int kClimberMotorRightPort = 40;
        public static final int kClimberMotorLeftPort = 41; 
      
        public static final double kClimberPowerUp = 1.0;
        public static final double kClimberPowerDown = -1.0;
        public static final double kClimberSlowUp = 0.1;
        public static final double kClimberSlowDown = -0.1;
      
        //testing on 2/12 showed something around 470k encoder ticks might be right
        public static final double kFullyClimbedTicks = 472000;
        public static final int kTimeoutMs = 30;
    }


    public static final class ShooterConstants {
        
        public static final int kShooterMotorPort1 = 30;
        public static final int kShooterMotorPort2 = 31;

        public static final TalonFXInvertType kShooterInversion1 = TalonFXInvertType.CounterClockwise;
        public static final TalonFXInvertType kShooterInversion2 = TalonFXInvertType.Clockwise;
        public static final boolean kSensorInversion = false;

        public static final double kShooterP = 100; // 0.15;
        public static final double kShooterI = 0.0;
        public static final double kShooterD = 0.0;
        public static final double kShooterF = 0.0135;

        public static final double kShooterCurrentLimit = 30.0;

        public static final double kShooterGearRatio = 42.0 / 42.0;

        public static final double kShooterRPMLowest = 3000;
        public static final double kShooterRPMLow = 3250;
        public static final double kShooterRPMMid = 3500;
        public static final double kShooterRPMHigh = 3750;
        public static final double kShooterRPMHighest = 4000;

        public static final int kPreShooterPort = 32;
        public static final int kFeederPort = 33;
        public static final TalonFXInvertType kPreShooterInversion = TalonFXInvertType.Clockwise;
        public static final boolean kFeederInversion = false;
        public static final double kPreShooterPercent = 0.5;
        public static final double kFeederPercent = 1.0;
        public static final double kSlowFeederPercent = 0.25;

        public static final double kPreShooterP = 0.15;
        public static final double kPreShooterI = 0;
        public static final double kPreShooterD = 0;
        public static final double kPreShooterF = 0.0135;

        public static final double kPreShooterIZ = 0;

        //TODO: preShooter gear ratio
        public static final double kPreShooterGearRatio = 1;

        public static final double kRPMWindow = 0.97;
        public static final double kPreShooterRPMWindow = 0.97;
    }


  
  public static final class IntakeConstants {
        public static final int kIntakeMotorPort = 20;
        public static final TalonFXInvertType kIntakeInversion = TalonFXInvertType.Clockwise;
        public static final double kIntakePercent = 0.50;
        public static final double kOutakePercent = 0.50;
    }

}
