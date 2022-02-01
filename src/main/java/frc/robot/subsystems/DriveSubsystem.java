package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

public class DriveSubsystem extends SubsystemBase {

        // initialize motors and drivetrain
        public final CANSparkMax frontLeftMotor = new CANSparkMax(Constants.DriveConstants.kFrontLeft,MotorType.kBrushless);
        public final CANSparkMax frontRightMotor = new CANSparkMax(Constants.DriveConstants.kFrontRight,MotorType.kBrushless);
        public final CANSparkMax rearLeftMotor = new CANSparkMax(Constants.DriveConstants.kRearLeft,MotorType.kBrushless);
        public final CANSparkMax rearRightMotor = new CANSparkMax(Constants.DriveConstants.kRearRight,MotorType.kBrushless);
        public final DifferentialDrive driveTrain = new DifferentialDrive(frontLeftMotor, frontRightMotor);
        private final DoubleSolenoid shifter = new DoubleSolenoid(solenoidType,shiftUp,shiftDown);
        // Sensor instantiations
        RelativeEncoder leftEncoder = rearLeftMotor.getEncoder();
        RelativeEncoder rightEncoder = frontRightMotor.getEncoder();
        PigeonIMU _pigeon;
        PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
        double[] ypr = new double[3];
        public Boolean imuActive = false;
        public Boolean autoDriving = false;
        private final DifferentialDriveOdometry odometry;
        private final PIDController pid = new PIDController(dP, dI, dD);


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
                odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(ypr[0]));
                leftEncoder.setPositionConversionFactor(kMPR);
                rightEncoder.setPositionConversionFactor(kMPR);
                _pigeon = new PigeonIMU(kGyro);
        }

        @Override
        public void periodic() {
                if (imuActive) {
                        _pigeon.getGeneralStatus(genStatus);
                        _pigeon.getYawPitchRoll(ypr);
                }
        }

        public void linearDriveMeters(double m) {
                
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
                if (t) shifter.set(DoubleSolenoid.Value.kReverse);
                else shifter.set(DoubleSolenoid.Value.kForward);
        }

        @Override
        public void simulationPeriodic() {
                // This method will be called once per scheduler run during simulation
        }
}
