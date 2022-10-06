package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.util.Color;


public final class Constants {
    public static final class DriveTrainConstants {// FIX THESE TOO
        public static final double kMaxSpeed = 2; // Meters per second
        public static final double kMaxAngularSpeed = Math.PI * 0.75 ; // 1/2 rotation per second

        public static final double driveSpeedP = 0.18;
        public static final double driveSpeedI = 0;
        public static final double driveSpeedD = 0;

        public static final double driveP = 0.1;
        public static final double driveI = 0;
        public static final double driveD = 0;

        public static final double turnP = 0.3;
        public static final double turnI = 0;
        public static final double turnD = 0;

        public static final double limeP = 0.08;
        public static final double limeI = 0;
        public static final double limeD = 0;

        public static final double driveMaxOutput = 0.8;
        public static final double turnMaxOutput = 0.45;

        public static final double driveEncoderMultiplier = (Math.PI * 0.102) / 14150;
        public static final double turnEncoderMultiplier = (2 * Math.PI) / 360;

        // CAN id
        public static final int frontLeftDrive = 6;
        public static final int frontLeftTurn = 5;
        public static final int frontLeftEncoder = 59;

        public static final int frontRightDrive = 4;
        public static final int frontRightTurn = 3;
        public static final int frontRightEncoder = 61;

        public static final int backLeftDrive = 8;
        public static final int backLeftTurn = 7;
        public static final int backLeftEncoder = 60;

        public static final int backRightDrive = 2;
        public static final int backRightTurn = 1;
        public static final int backRightEncoder = 62;

        public static final int pigeon = 24;
    }

    public static class ControllerConstants { // FIX VALUES SUJAY
        public static final int LEFT_X = 0;
        public static final int LEFT_Y = 1;
        public static final int RIGHT_X = 0;
        public static final int RIGHT_Y = 0;

        public static final int YELLOW_BUTTON = 4;
        public static final int RED_BUTTON = 2;
        public static final int GREEN_BUTTON = 1;
        public static final int BLUE_BUTTON = 3;

        public static final int LEFT_TRIGGER = 0;
        public static final int RIGHT_TRIGGER = 0;
        public static final int LEFT_BUMPER = 0;
        public static final int RIGHT_BUMPER = 0;
    }
}