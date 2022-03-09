package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import static frc.robot.Constants.DriveConstants.*;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;

public class DriveSubsystem extends SubsystemBase {

        // initialize motors and drivetrain
        public final CANSparkMax frontLeftMotor = new CANSparkMax(Constants.DriveConstants.kFrontLeft,
                        MotorType.kBrushless);
        public final CANSparkMax frontRightMotor = new CANSparkMax(Constants.DriveConstants.kFrontRight,
                        MotorType.kBrushless);
        public final CANSparkMax rearLeftMotor = new CANSparkMax(Constants.DriveConstants.kRearLeft,
                        MotorType.kBrushless);
        public final CANSparkMax rearRightMotor = new CANSparkMax(Constants.DriveConstants.kRearRight,
                        MotorType.kBrushless);
        public final DifferentialDrive driveTrain = new DifferentialDrive(frontLeftMotor, frontRightMotor);
        private final DoubleSolenoid shifter = new DoubleSolenoid(2, solenoidType, shiftUp, shiftDown);
        // Sensor instantiations
        RelativeEncoder leftEncoder = rearLeftMotor.getEncoder();
        RelativeEncoder rightEncoder = frontRightMotor.getEncoder();
        PigeonIMU _pigeon;
        PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
        // odometry
        double[] ypr = new double[3];
        Pose2d pos = new Pose2d();
        DifferentialDriveOdometry odometry;

        public DriveSubsystem() {
                // drivetrain
                /*
                frontLeftMotor.restoreFactoryDefaults();
                frontRightMotor.restoreFactoryDefaults();
                rearLeftMotor.restoreFactoryDefaults();
                rearRightMotor.restoreFactoryDefaults();*/

                frontLeftMotor.setInverted(true);
                frontRightMotor.setInverted(false);
                rearLeftMotor.follow(frontLeftMotor);// front left yields faulty encoder values so that set follower
                rearRightMotor.follow(frontRightMotor);
                leftEncoder.setPositionConversionFactor(kMPR);
                rightEncoder.setPositionConversionFactor(kMPR);
                odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getRawHeading()));
                _pigeon = new PigeonIMU(kGyro);
        }

        double lastYaw = 0;
        @Override
        public void periodic() {
                updateIMU();
                updateOdometry();
                updateWidgets();
                lastYaw = ypr[0];
        }

        public void updateWidgets() {
                mainHeading.setDouble(getPose().getRotation().getDegrees());
                mainX.setDouble(getPose().getX());
                mainY.setDouble(getPose().getY());
        }

        private final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
        private final NetworkTableEntry mainHeading = mainTab.add("heading",0).withPosition(3, 3).withSize(1, 1).getEntry();
        private final NetworkTableEntry mainX = mainTab.add("x",0).withPosition(3, 1).withSize(1,1).getEntry();
        private final NetworkTableEntry mainY = mainTab.add("y",0).withPosition(3, 2).withSize(1,1).getEntry();

        public void resetEncoders() {
                rightEncoder.setPosition(0.0);
                leftEncoder.setPosition(0.0);
        }

        // arcade drive method to be called by commands
        public void arcadeDrive(Double forwardPower, Double turnPower) {
                driveTrain.arcadeDrive(forwardPower, turnPower);
        }

        // tank drive method to be called by commands
        public void tankDrive(Double leftPower, Double rightPower) {
                driveTrain.tankDrive(leftPower, rightPower);
        }

        public void setHighGear(Boolean t) {
                if (!t) shifter.set(DoubleSolenoid.Value.kReverse);
                else shifter.set(DoubleSolenoid.Value.kForward);
        }

        public Pose2d getPose() {
                return odometry.getPoseMeters();
        }

        public void resetOdometry(Pose2d pose) {
                resetEncoders();
                odometry.resetPosition(pose, Rotation2d.fromDegrees(getRawHeading()));
        }

        public double getRawHeading() {
                double y = -ypr[0];
                while (y < 0)
                        y += 360;
                while (y > 360)
                        y -= 360;
                return y;
        }

        public double getHeadingError(double sp) {
                double v = sp - getPose().getRotation().getDegrees();
                while (v < -180)
                        v += 360;
                while (v > 180)
                        v -= 360;
                return v;
        }

        private void updateIMU() {
                _pigeon.getGeneralStatus(genStatus);
                _pigeon.getYawPitchRoll(ypr);
        }

        private void updateOdometry() {
                odometry.update(Rotation2d.fromDegrees(getRawHeading()), leftEncoder.getPosition(),
                                rightEncoder.getPosition());
        }

        public double getRotationSpeed() {
                return Math.abs(lastYaw - ypr[0])*50;
        }
}
