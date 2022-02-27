// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ControllerConstants {

        public static final int kLeftVertical = 1;
        public static final int kRightVertical = 5;
        public static final int kLeftHorizontal = 0;
        public static final int kRightHorizontal = 4;
        public static final int kLeftTrigger = 2;
        public static final int kRightTrigger = 3;

        public static final int kA = 1;
        public static final int kB = 2;
        public static final int kX = 3;
        public static final int kY = 4;
        public static final int kLeftBumper = 5;
        public static final int kRightBumper = 6;
        public static final int kLeftOptions = 7;
        public static final int kRightOptions = 8;
        public static final int kLeftJoystickPressed = 9;
        public static final int kRightJoystickPressed = 10;
    }

    public static final class LimeLightConstants {
        public static final double targetLostWaitTime = 0.15;
        public static final double targetHeight = 2.6416;
        public static final double mountHeight = 0.9;
        public static final double mountAngle = 40;
    }

    public static final class BigIronConstants {
        public static final int kDrumOneID = 11;
        public static final int kDrumTwoID = 12;
        public static final int kDrumDirection = -1;
        public static final int kDrumIdleSpeed = 2000;
        public static final int kDrumSpeedTolerance = 40;
        public static final int kHoodDownLimitPin = 9;
        public static final int kBreachSensorPin = 0;
        public static final int kIntakeColorSensorThreshold = 140;
        public static final double kHoodPositionTolerance = 0.01;
        public static final ColorSensorV3 intakeSensor = new ColorSensorV3(I2C.Port.kMXP);
        public static final int kIntakeID = 30;
        public static final int kHoodID = 31;
        public static final int kBeltID = 32;
        public static final int kHoodEncoderPin = 8;
        public static final double kIntakePower = -0.9;
        public static final double kIntakeReversePower = 0.2;
        public static final double kBeltReversePower = 0.5;
        public static final double kBeltPower = -0.9;
        public static final double HP = 10;
        public static final double HI = 0.5;
        public static final double HD = 0;
        public static final double HC = 1;
        public static final double DP = 0.00043;
        public static final double DI = 0.0005;
        public static final double DD = 0.00001;
        public static final int kChannelIntakeForwardGo = 7;
        public static final int kChannelIntakeForwardVent = 5;
        public static final int kChannelIntakeBackwardGo = 6;
        public static final int kChannelIntakeBackwardVent = 4;
    }

    public static final class DriveConstants {
        public static final int kFrontLeft = 2; // CAN (Spark)
        public static final int kFrontRight = 3; // CAN (Spark)
        public static final int kRearLeft = 4; // CAN (Spark)
        public static final int kRearRight = 5; // CAN (Spark)
        public static final PneumaticsModuleType solenoidType = PneumaticsModuleType.CTREPCM;
        public static final int shiftUp = 2;
        public static final int shiftDown = 3;
        public static final double kSmoothingAccelFactor = 0.13;
        public static final double kSmoothingDecelFactor = 0.03;
        public static final double kMaxDownshiftPower = 0.35;
        public static final double kMPR = 0.0207;// meters per revolution
        public static final int kGyro = 10;

        public static final double ltP = 0.016;
        public static final double ltI = 0.023;
        public static final double ltD = 0.0055;

        public static final double tP = 0.018;// P
        public static final double tI = 0.0019;// I
        public static final double tD = 0.0028;// D
        public static final double tC = 0.54;// Clamp //t and d are two different PID controllers
        public static final double[] kDriveDistances = {0,1,2,3,4,30};
        public static final double[] kDrivePowers = {0.0,0.12,0.6,0.8,1,1};
        public static final double kAutoDriveSmoothing = 0.06;
        public static final double dP = 0.3;
        public static final double dI = 0.2;
        public static final double dD = 0;
        public static final double dC = 1;
    }

    public static final class ShooterData {
        /*
        The distance MUST be greater at higher indexes, and by GOD 
        don't make neighboring distance values the same, or Java
        Satan himself will reject you to be abandoned in the Endless Sea
        of DBZ, aboard a raft equipped with nothing but a Chromebook.
        */
        public static final double[] distances = {4.06,5.03,5.97};
        public static final int[] drumSpeeds = {3550,3600,3950};
        public static final double[] hoodPositions = {0.1,0.3,0.5};
    }
}
