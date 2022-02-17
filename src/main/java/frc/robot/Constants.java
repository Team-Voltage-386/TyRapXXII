// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
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

    public static final class BigIronConstants {
        public static final int kDrumLeadID = 0;
        public static final int kDrumFollowID = 0;
        public static final int kDrumIdleSpeed = 2800;
        public static final int kDrumSpeedTolerance = 75;
        public static final double kHoodPositionTolerance = 0.05;
        public static final int kIntakeID = 9;
        public static final double kIntakePower = -0.9;
        public static final double HP = 0;
        public static final double HI = 0;
        public static final double HD = 0;
        public static final double HC = 1;
        public static final double DP = 0;
        public static final double DI = 0;
        public static final double DD = 0;
    }

    public static final class DriveConstants {
        public static final int kFrontLeft = 2; // CAN (Spark)
        public static final int kFrontRight = 3; // CAN (Spark)
        public static final int kRearLeft = 4; // CAN (Spark)
        public static final int kRearRight = 5; // CAN (Spark)
        public static final PneumaticsModuleType solenoidType = PneumaticsModuleType.CTREPCM;
        public static final int shiftUp = 2;
        public static final int shiftDown = 3;
        public static final double kMPR = 0.4788;
        public static final TalonSRX kGyro = new TalonSRX(9);

        public static final double dP = 0.1;
        public static final double dI = 0;
        public static final double dD = 0.001;
        public static final double dC = 0.4;
    }

    public static final class ShooterData {

    }
}
