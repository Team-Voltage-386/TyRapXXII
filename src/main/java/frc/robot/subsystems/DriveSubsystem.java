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
        public final CANSparkMax frontLeftMotor = new CANSparkMax(Constants.DriveConstants.kFrontLeft,MotorType.kBrushless);
        public final CANSparkMax frontRightMotor = new CANSparkMax(Constants.DriveConstants.kFrontRight,MotorType.kBrushless);
        public final CANSparkMax rearLeftMotor = new CANSparkMax(Constants.DriveConstants.kRearLeft,MotorType.kBrushless);
        public final CANSparkMax rearRightMotor = new CANSparkMax(Constants.DriveConstants.kRearRight,MotorType.kBrushless);
        public final DifferentialDrive driveTrain = new DifferentialDrive(frontLeftMotor, frontRightMotor);
        //private final DoubleSolenoid shifter = new DoubleSolenoid(solenoidType,shiftUp,shiftDown);
        // Sensor instantiations
        RelativeEncoder leftEncoder = rearLeftMotor.getEncoder();
        RelativeEncoder rightEncoder = frontRightMotor.getEncoder();
        PigeonIMU _pigeon;
        PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
        double[] ypr = new double[3];


        private ShuffleboardTab tab = Shuffleboard.getTab("Drive");

        // Create output widgets
        private NetworkTableEntry frontLeftOutputWidget = tab.add("F-L Output", 0).withPosition(0, 0).getEntry();
        private NetworkTableEntry frontRightOutputWidget = tab.add("F-R Output", 0).withPosition(1, 0).getEntry();
        private NetworkTableEntry backLeftOutputWidget = tab.add("B-L Output", 0).withPosition(0, 1).getEntry();
        private NetworkTableEntry backRightOutputWidget = tab.add("B-R Output", 0).withPosition(1, 1).getEntry();

        // Create temperature widgets
        private NetworkTableEntry frontLeftTempWidget = tab.add("F-L Temp", 0).withPosition(3, 0).getEntry();
        private NetworkTableEntry frontRightTempWidget = tab.add("F-R Temp", 0).withPosition(4, 0).getEntry();
        private NetworkTableEntry backLeftTempWidget = tab.add("B-L Temp", 0).withPosition(3, 1).getEntry();
        private NetworkTableEntry backRightTempWidget = tab.add("B-R Temp", 0).withPosition(4, 1).getEntry();

        // Create current widgets
        private NetworkTableEntry frontLeftCurrentWidget = tab.add("F-L Current", 0).withPosition(6, 0).getEntry();
        private NetworkTableEntry frontRightCurrentWidget = tab.add("F-R Current", 0).withPosition(7, 0).getEntry();
        private NetworkTableEntry backLeftCurrentWidget = tab.add("B-L Current", 0).withPosition(6, 1).getEntry();
        private NetworkTableEntry backRightCurrentWidget = tab.add("B-R Current", 0).withPosition(7, 1).getEntry();

        // Create encoder widgets
        private NetworkTableEntry leftEncoderWidget = tab.add("Left Encoder", 0).withSize(2, 1).withPosition(2, 2)
                        .getEntry();
        private NetworkTableEntry rightEncoderWidget = tab.add("Right Encoder", 0).withSize(2, 1).withPosition(4, 2)
                        .getEntry();


                        


        public DriveSubsystem() {
                // drivetrain
                frontLeftMotor.restoreFactoryDefaults();
                frontRightMotor.restoreFactoryDefaults();
                rearLeftMotor.restoreFactoryDefaults();
                rearRightMotor.restoreFactoryDefaults();

                frontLeftMotor.setInverted(true);
                frontRightMotor.setInverted(false);
                rearLeftMotor.follow(frontLeftMotor);// front left yields faulty encoder values so that set follower
                rearRightMotor.follow(frontRightMotor);
                leftEncoder.setPositionConversionFactor(kMPR);
                rightEncoder.setPositionConversionFactor(kMPR);
                _pigeon = new PigeonIMU(kGyro);
        }

        @Override
        public void periodic() {
                // Update output widgets
                frontLeftOutputWidget.setDouble(frontLeftMotor.get());
                frontRightOutputWidget.setDouble(frontRightMotor.get());
                backLeftOutputWidget.setDouble(rearLeftMotor.get());
                backRightOutputWidget.setDouble(rearRightMotor.get());

                // Update temp widgets
                frontLeftTempWidget.setDouble(frontLeftMotor.getMotorTemperature());
                frontRightTempWidget.setDouble(frontRightMotor.getMotorTemperature());
                backLeftTempWidget.setDouble(rearLeftMotor.getMotorTemperature());
                backRightTempWidget.setDouble(rearRightMotor.getMotorTemperature());

                // Update current widgets
                frontLeftCurrentWidget.setDouble(frontLeftMotor.getOutputCurrent());
                frontRightCurrentWidget.setDouble(frontRightMotor.getOutputCurrent());
                backLeftCurrentWidget.setDouble(rearLeftMotor.getOutputCurrent());
                backRightCurrentWidget.setDouble(rearRightMotor.getOutputCurrent());

                // Update encoder widgets
                leftEncoderWidget.setDouble(leftEncoder.getPosition());
                rightEncoderWidget.setDouble(rightEncoder.getPosition());
        }

        public void resetEncoders() {
                rightEncoder.setPosition(0.0);
                leftEncoder.setPosition(0.0);
        }

        // arcade drive method to be called by commands
        public void arcadeDrive(Double forwardPower, Double turnPower) {
                driveTrain.arcadeDrive(forwardPower, turnPower);
        }

        //tank drive method to be called by commands
        public void tankDrive(Double leftPower, Double rightPower) {
                driveTrain.tankDrive(leftPower, rightPower);
        }

        public void setHighGear(Boolean t) {
                //if (t) shifter.set(DoubleSolenoid.Value.kReverse);
                //else shifter.set(DoubleSolenoid.Value.kForward);
        }
}
