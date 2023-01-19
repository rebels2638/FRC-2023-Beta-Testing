// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

//I think we needed this somewhat made but I still need to fix some things as this is based on github and not whats on the current pc
//so just tell me when something gets updated and i'll add it
public final class Constants {

    public static class DriveConstants {
        public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 2;
        public static final int kRightMotor1Port = 3;
        public static final int kRightMotor2Port = 4;

        // The following are test values - they will need to be adjusted for robot!
        public static final double kMaxSpeed = 3.0; // meters per second
        public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

        public static final double kTrackWidth = 0.381 * 2; // meters
        public static final double kWheelRadius = 0.0508; // meters

        // encoder constants [a, b]
        public static final int kLeftEncoderPorts[] = {0, 1};
        public static final int kRightEncoderPorts[] = {2, 3};
        public static final int kEncoderResolution = 4096;
        public static final double kEncoderDistancePerPulse = 2 * Math.PI * kWheelRadius / kEncoderResolution;

        //gyro port
        public static final int kGyroPort = 0;

        //PID controller gains
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        //feed forward gains (garbage values currently) NEED FIX!!
        public static final double kS = 1.0;
        public static final double kV = 3.0;
        public static final double kA = 0.0;

        //xbox controller ports
        public static final int kXboxDriverPort = 3;
        public static final int kXboxOperatorPort = 2;
        public static class ElevatorConstants {
        public static final int kElevatorId = 5;
    }
    
    public static class ClawConstants {
        public static final int kClawId = 6;
    }

    public static class GyroConstants {
        public static final int kGyroPort = 0;
        public static final double kThreshold = 0.5;
        public static final double kGoal = 0;
        public static final double kLimitPower = 0.5; 
        public static final double kPConstant = 0.02; // "please work, oh please work. I beg of you"
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }
}

