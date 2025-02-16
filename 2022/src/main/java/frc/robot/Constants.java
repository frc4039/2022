// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int kPCMCANID = 51;

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;


    public static final double kDeadband = 0.075;
    public static final double kRotationScale = 0.25;
    public static final double kRotationOfHub = 68;

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

    public static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = -Math.toRadians(200.3);
    public static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = -Math.toRadians(256.8);
    public static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = -Math.toRadians(337.1);
    public static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = -Math.toRadians(205.4);

    public static final double GLASS_OFFSET_X = 10;
    public static final double GLASS_OFFSET_Y = 5;
    public static final double GLASS_SCALE = 0.0254;

    public static final double CONTROLLER_ROTATION_DEADBAND = 0.98;
    public static final double LIMELIGHT_ROTATION_FEEDFORWARD = 0.00125;

    public static final double DRIVETRAIN_DRIVE_CURRENT_LIMIT = 50;


    public static final class ClimberConstants{
        public static final TalonFXInvertType kClimberMotorRightInversion = TalonFXInvertType.CounterClockwise;
        public static final TalonFXInvertType kClimberMotorLeftInversion = TalonFXInvertType.Clockwise;
        public static final int kClimberMotorRightPort = 40;
        public static final int kClimberMotorLeftPort = 41; 
      
        public static final double kClimberPowerUp = 1.0;
        public static final double kClimberPowerDown = -1.0;
        public static final double kClimberSlowUp = 0.2;
        public static final double kClimberSlowDown = -0.2;
      
        public static final double kFullyClimbedTicks = 300000;
        public static final int kTimeoutMs = 30;
        
        public static final double kClimberVelocityUp = 200.5;
        public static final double kClimberVelocityDown = -200.5;

        public static final double kClimberF = 0;
        public static final double kClimberP = 0;
        public static final double kClimberI = 0;
        public static final double kClimberD = 0;

        public static final int kRightBottomLimitSwitchPort = 1;
        public static final int kLeftBottomLimitSwitchPort = 3;
        public static final int kRightTopBreakBeamPort = 0;
        public static final int kLeftTopBreakBeamPort = 2;

        public static final double kLeftClimberPreClimb = 40000;
        public static final double kRightClimberPreClimb = 40000;
    }



    public static final class ShooterConstants {
        
        public static final int kShooterMotorPort1 = 30;
        public static final int kShooterMotorPort2 = 31;

        public static final double kShooterRPM = 2100;
        public static final double kPreShooterRPM = 500;
        public static final TalonFXInvertType kShooterInversion1 = TalonFXInvertType.CounterClockwise;
        public static final TalonFXInvertType kShooterInversion2 = TalonFXInvertType.Clockwise;
        public static final boolean kSensorInversion = false;

        public static final double kShooterP = 0.02;
        public static final double kShooterI = 0.0;
        public static final double kShooterD = 0.0;
        public static final double kShooterF = 0.055;

        public static final double kShooterCurrentLimit = 30.0;

        public static final double kShooterGearRatio = 42.0 / 42.0;

        public static final int kPreShooterPort = 32;
        public static final TalonFXInvertType kPreShooterInversion = TalonFXInvertType.Clockwise;
        public static final boolean kPreShooterSensorInversion = false;
        public static final double kPreShooterPercent = 0.5;

        public static final double kPreShooterP = 0.025;
        public static final double kPreShooterI = 0;
        public static final double kPreShooterD = 0;
        public static final double kPreShooterF = 0.05;

        //TODO: preShooter gear ratio
        public static final double kPreShooterGearRatio = 3.0/1.0;

        public static final double kfenderLowShotRPM = 1150;
        public static final double kfenderHighShotRPM = 2100;
        public static final double klimelightShotRPM = 2500;
        
        public static final double kpreShooterReverseRPM = 500;
        public static final double kpreShooterLimelightShotRPM = 2000;
        public static final double kpreShooterFenderLowShotRPM = 500;
        public static final double kpreShooterFenderHighShotRPM = 500;

        public static final double kfenderHighShotRPMWindow = 0.05;
        public static final double kPreShooterFenderHighShotRPMWindow = 0.05;
        public static final double kfenderLowShotRPMWindow = 0.2;
        public static final double kPreShooterFenderLowShotRPMWindow = 0.2;
        public static final double klimelightShotRPMWindow = 0.05;
        public static final double kPreShooterlimelightShotRPMWindow = 0.05;

        public static final double kClosestKey = 105.0;
        public static final double kCloseKey = 135.0;
        public static final double kFarKey = 185.0;
        public static final double kFarthestKey = 250.0;
        public static final double kClosestValue = 2100.0;
        public static final double kCloseValue = 2300.0;
        public static final double kFarValue = 2625.0;
        public static final double kFarthestValue = 3120.0;
        public static final double kAngleWindow = 10;
        public static final double kShotAngularVelocityLimit = 5;

    }


    public static final class FeederConstants {
        
        public static final int kFeederPort = 33;
        public static final boolean kFeederInversion = false;
        public static final double kFeederFeedPercent = 0.25;
        public static final double kFeederEjectPercent = 0.8;
        public static final double kFeederHighShotPercent = 0.35;
        public static final double kFeederLowShotPercent = 1;
        public static final double kFeederLimelightShotPercent = 1;


        public static final int kBreakBeamPreShooterPort = 9;
        public static final int kBreakBeamLowerBall = 7;
        public static final int kBreakBeamUpperBall = 8;
        public static final int kBreakBeamIntakePort = 6;
    }


    public static final class IntakeConstants {
        public static final int kIntakeMotorPort = 20;
        public static final TalonFXInvertType kIntakeInversion = TalonFXInvertType.Clockwise;
        public static final double kIntakePercent = 0.50;
        public static final double kOutakePercent = 0.50;
    }


    public static final class LEDConstants {
        public static final int LED_STRIP_PORT = 0;
        public static final int LED_STRIP_LENGTH = 123;
        //Brightness and Saturation are a 0-255 scale
        public static final int BRIGHTNESS = 80;
        public static final int SATURATION = 255;
        //rainbow hue speed is the speed for the slowest rainbow
        public static final double RAINBOW_HUE_SPEED = 0.5;
        //Hue is 0-360
        public static final double FLASHING_SPEED = 0.5;
        //the rate in seconds that flashing hue flashes at

        public static final int HUE_RED = 0;
        public static final int HUE_ORANGE = 30;
        public static final int HUE_YELLOW = 60;
        public static final int HUE_GREEN = 120;
        public static final int HUE_CYAN = 180;
        public static final int HUE_BLUE = 240;
        public static final int HUE_VIOLET = 270;
        public static final int HUE_MAGENTA = 300;
    }
}
