// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int XBOX_OPERATOR_PORT = 2;
    public static final int XBOX_DRIVER_PORT = 3;

    public static final double FALCON_CPR = 2048;

    public static final class DriveConstants {
        public static enum DriveTypes {
            CURVATURE, TANK
        };

        // Left gearbox
        public static final int FALCON_LEFT_FRONT_ID = 13; // 13
        public static final int FALCON_LEFT_BACK_ID = 12; // 12
        public static final boolean FALCON_LEFT_FRONT_INVERTED = true;
        public static final boolean FALCON_LEFT_BACK_INVERTED = true;

        // Right gearbox
        public static final int FALCON_RIGHT_FRONT_ID = 14; // 14
        public static final int FALCON_RIGHT_BACK_ID = 15; // 15
        public static final boolean FALCON_RIGHT_FRONT_INVERTED = false;
        public static final boolean FALCON_RIGHT_BACK_INVERTED = false;

        // Solenoid
        public static final int PCM_ID = 0;
        public static final int SOLENOID_FORWARD = 6;
        public static final int SOLENOID_REVERSE = 1;

        // Gyro
        public static final boolean GYRO_REVERSED = true;

        // Differential Drive Kinematics
        public static final double TRACK_WIDTH_METERS = 0.6096;
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_METERS);

        // Curvature drive
        public static final double DEADBAND = 0.05;

        public static final DriveTypes DRIVE_TYPE_DEFAULT = DriveTypes.CURVATURE;
        public static final double QUICK_TURN_THRESHOLD = 0.05;

        public static final double ANGLE_TOLERANCE = 4.0;

        // Slew Rate Limit
        public static final double SLEW_RATE_LIMIT = 1.0;

        // Pose estimation constants
        public static final Matrix<N5, N1> POSE_STATE_MEASUREMENT_STDEV = new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02,
                0.02, 0.01, 0.02, 0.02);
        public static final Matrix<N3, N1> POSE_LOCAL_MEASUREMENT_STDEV = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02,
                0.02, 0.01);
        public static final Matrix<N3, N1> POSE_GLOBAL_MEASUREMENT_STDEV = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1,
                0.1, 0.01);

        public static final double TURN_SENSITIVITY = .69;
    }

    public static final class GearboxConstants {
        public static enum PIDTypes {
            VELOCITY, MOTION
        };

        public static final double PID_P = 1.3523; // 61.128; // 2.83e-6 // 0.0695; 2.83e-5; 2.3e-5
        public static final double PID_I = 0.0;
        public static final double PID_D = 110.34; // 8.1816;
        public static final double PID_F = 0.0;

        public static final double VOLTAGE_COMP_SATURATION = 12.0;
        public static final boolean VOLTAGE_COMP_ENABLED = true;

        public static final double WHEEL_DIAMETER = 0.127;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        /**
         * Motor Clicks Per Rotation / gear reduction = effective CPR
         **/
        //public static final double HIGH_GEAR_REDUCTION = 222302 / (2.3114 * 5.08) / FALCON_CPR; // This year? 5.39; // Last year: 20.83
        public static final double HIGH_GEAR_EFFECTIVE_CPR = 2048;// FALCON_CPR * HIGH_GEAR_REDUCTION;

        //public static final double LOW_GEAR_REDUCTION = 222302 / (2.3114 * 5.08) / FALCON_CPR; // This year? 12.25; // Last year: 9.17
        public static final double LOW_GEAR_EFFECTIVE_CPR = 2048; // FALCON_CPR * LOW_GEAR_REDUCTION;

        // Feed forward/back gains
        public static final double STATIC_GAIN = 0.72979; // volts
        public static final double VELOCITY_GAIN = 2.7527; // volt seconds per meter
        public static final double ACCEL_GAIN = 0.52121; // volt seconds squared per meter

        public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(STATIC_GAIN, VELOCITY_GAIN,
                ACCEL_GAIN);

        // motion magic
        // public static final double MOTION_P = 0.2;
        // public static final double MOTION_I = 0.0;
        // public static final double MOTION_D = 0.0;
        // public static final double MOTION_F = 0.2;
        public static final double MOTION_CRUISE_VELOCITY = 1;
        public static final double MOTION_ACCELERATION = .5;
        public static final double MOTION_TOLERANCE = 200;

        public static final NeutralMode DEFAULT_NEUTRAL_MODE = NeutralMode.Brake;
        public static final double MOTION_P = 0;
    }

    public static final class IntakeConstants {
        public static final int TALON_ID = 5;

        public static final PneumaticsModuleType SOLENOID_MODULE_TYPE = PneumaticsModuleType.CTREPCM;
        public static final int SOLENOID_FORWARD = 7;
        public static final int SOLENOID_REVERSE = 0;

        public static final double EMPTY_INTAKE_SPEED = 0.75;
        public static final double FULL_INTAKE_SPEED = 1;
    }

    public static final class AdjustableClimberConstants {
        public static final PneumaticsModuleType SOLENOID_MODULE_TYPE = PneumaticsModuleType.CTREPCM;
        public static final int SOLENOID_FORWARD = 4;
        public static final int SOLENOID_REVERSE = 2;

    }

    public static final class ShooterConstants {
        // Shooter
        public static final int SHOOTER_ID = 17;
        public static final boolean SHOOTER_INVERTED = false;

        public static final NeutralMode SHOOTER_NEUTRAL_MODE = NeutralMode.Coast;

        public static final double LOW_GOAL_HOPPER_PERCENTAGE = 0.3;
        public static final double LOW_GOAL_SHOOTER_PERCENTAGE = 0.3;

        public static final double HIGH_GOAL_HOPPER_PERCENTAGE = 0.8;//0.35;
        public static final double HIGH_GOAL_SHOOTER_PERCENTAGE = 0.72;

        public static final int HOPPER_DELAY_TIME = 30;

        public static final int HIGH_HOPPER_DELAY_TIME = 90;

        public static final double SHOOTER_GEAR_REDUCTION = 1.0;
        public static final double SHOOTER_EFFECTIVE_CPR = FALCON_CPR * SHOOTER_GEAR_REDUCTION;

        public static final double ACCELERATOR_GEAR_REDUCTION = 1.0;
        public static final double ACCELERATOR_EFFECTIVE_CPR = FALCON_CPR * ACCELERATOR_GEAR_REDUCTION;

        public static final int SHOOTER_MAX_RPM = 6830;
        public static final double SHOOTER_CLOSED_LOOP_RAMP = 0.0;

        // Shooter Velocity PIDF values
        public static final double SHOOTER_PID_P = 0.08; //0.072277;
        public static final double SHOOTER_PID_I = 0.0;
        public static final double SHOOTER_PID_D = 0.0;

        // Hood Encoder
        // - PIDF controller takes in target and sensor position measurements in “raw”
        // sensor units. This means a CTRE Mag Encoder will count 4096 units per
        // rotation.
        // - PIDF controller takes in target and sensor velocity measurements in “raw”
        // sensor units per 100ms.
        // - PIDF controller calculates the motor output such that, 1023 is interpreted as
        // “full”. This means a closed loop error of 341 (sensor units) X kP of 3.0 will
        // produce full motor output (1023
        public static final int ENCODER_UNITS_PER_ROTATION = 2048;

        // Hood
        public static final int HOOD_ID = 1;
        public static final boolean HOOD_INVERTED = true;
        public static final NeutralMode HOOD_NEUTRAL_MODE = NeutralMode.Brake;
        public static final double Min_HOOD_PERCENTAGE_SPEED = 0.15;
        public static final double MAX_HOOD_PERCENTAGE_SPEED = 0.2;
        public static final double MAX_ANGLE = 360.0;
        public static final double ANGLE_THRESHOLD = 15.0;

        // Hood Limit Switch
        public static final int HOOD_LIMIT_SWITCH_ID = 9; 

        // Goal Constants
        public static final double GOAL_HEIGHT = 2.64;

        public static final double DEFAULT_SPEED = 1;
        public static final double RPM_MAX_ERROR = 10;

        public static final double STATIC_GAIN = 0.02146;
        public static final double VELOCITY_GAIN = 1.2061;
        public static final double ACCEL_GAIN = 0.117;

        public static final SimpleMotorFeedforward SHOOTER_FF = new SimpleMotorFeedforward(STATIC_GAIN, VELOCITY_GAIN,
                ACCEL_GAIN);

        public static final double SHOOTER_VOLTAGE_COMP_SATURATION = 12.0;
        public static final boolean SHOOTER_VOLTAGE_COMP_ENABLED = true;

        public static final double HIGH_GOAL_SHOOTER_HIGH_VELO = 11000;
        public static final double HIGH_GOAL_SHOOTER_HIGH_THRESHOLD = 1.36;
        
        public static final double HIGH_GOAL_SHOOTER_LOW_VELO = 11000;
        public static final double HIGH_GOAL_SHOOTER_LOW_THRESHOLD = 1.36;

        
        public static final double HIGH_DIST = 100;

        public static final double LOW_DIST = 100;
    }

    public static final class ClimberConstants {
        public static final int LEFT_NEO_ID = 20;
        public static final int RIGHT_NEO_ID = 21;
        public static final boolean LEFT_INVERTED = false;
        public static final boolean RIGHT_INVERTED = true;

        public static final double DEADBAND = 0.05;

        public static final int TOP_LEFT_LIMIT_SWITCH = 2;
        public static final int BOTTOM_LEFT_LIMIT_SWITCH = 1;

        public static final int TOP_RIGHT_LIMIT_SWITCH = 4;
        public static final int BOTTOM_RIGHT_LIMIT_SWITCH = 3;

        // TODO: bro im just randomly putting numbers here
        public static final double P = 0.1;
        public static final double I = 1e-4;
        public static final double D = 1.0;
        public static final double FF = 0.0;
        public static final double IZONE = 0.0;
        public static final double MAX_VELOCITY = 10.0;
        public static final double MIN_VELOCITY = 0.0;
        public static final double MAX_ACCELERATION = 10.0;
        public static final double ALLOWED_ERROR = 5.0;
        public static final int TIMEOUT_MS = 5;
        
    }

    public static final class HoodConstants {
        public static final PneumaticsModuleType SOLENOID_MODULE_TYPE = PneumaticsModuleType.CTREPCM;
        public static final int SOLENOID_FORWARD = 5;
        public static final int SOLENOID_REVERSE = 3;
    }

    public static final class HopperConstants {
        public static final int BELT_TALON_ID = 3;
        public static final boolean BELT_TALON_INVERTED = false;

        public static final int LOWER_LINE_BREAK_PORT = 8;
        public static final int HIGHER_LINE_BREAK_PORT = 6;

        public static final double BRING_BACK_SPEED = 0.2;
    }

    public static final class AutoConstants {
        // Timer for Auto Start (in 20 ms)
        public static final int AUTO_START_TIMER = 0;

        // Baseline values for RAMSETE follower in units of meters and seconds
        public static final double RAMSETE_B = 2; // before 1.5
        public static final double RAMSETE_ZETA = 0.7; // before 1.4
    }
}