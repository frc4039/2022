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
  
  public static final class IntakeConstants {
        public static final int kIntakeMotorPort = 20;
        public static final TalonFXInvertType kIntakeInversion = TalonFXInvertType.Clockwise;
        public static final double kIntakePercent = 0.75;

    }

}
