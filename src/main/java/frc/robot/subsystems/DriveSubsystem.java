package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static frc.robot.Constants.DriveConstants.*;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableEntry;

/** TyRapXXII's drive subsystem
 * @author Max V.
 * @author Carl C.
 */
public class DriveSubsystem extends SubsystemBase {

        // initialize motors and drivetrain
        public final CANSparkMax frontLeftMotor = new CANSparkMax(Constants.DriveConstants.kFrontLeft, MotorType.kBrushless);
        public final CANSparkMax frontRightMotor = new CANSparkMax(Constants.DriveConstants.kFrontRight, MotorType.kBrushless);
        public final CANSparkMax rearLeftMotor = new CANSparkMax(Constants.DriveConstants.kRearLeft, MotorType.kBrushless);
        public final CANSparkMax rearRightMotor = new CANSparkMax(Constants.DriveConstants.kRearRight, MotorType.kBrushless);
        public final DifferentialDrive driveTrain = new DifferentialDrive(frontLeftMotor, frontRightMotor);
        private final DoubleSolenoid shifter = new DoubleSolenoid(2, solenoidType, kShiftUp, kShiftDown);
        // Sensor instantiations
        RelativeEncoder leftEncoder = rearLeftMotor.getEncoder();
        RelativeEncoder rightEncoder = frontRightMotor.getEncoder();
        PigeonIMU _pigeon;
        PigeonIMU.GeneralStatus genStatus = new PigeonIMU.GeneralStatus();
        // odometry
        double[] ypr = new double[3];
        Pose2d pos = new Pose2d();
        DifferentialDriveOdometry odometry;
        /** creates a new DriveSubsystem */
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

        /** updates the widgets on the "Main" tab */
        public void updateWidgets() {
                mainHeading.setDouble(getPose().getRotation().getDegrees());
                mainX.setDouble(getPose().getX());
                mainY.setDouble(getPose().getY());
        }

        private final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
        private final NetworkTableEntry mainHeading = mainTab.add("heading",0).withPosition(3, 3).withSize(1, 1).getEntry();
        private final NetworkTableEntry mainX = mainTab.add("x",0).withPosition(3, 1).withSize(1,1).getEntry();
        private final NetworkTableEntry mainY = mainTab.add("y",0).withPosition(3, 2).withSize(1,1).getEntry();

        /** resets the encoder positions */
        public void resetEncoders() {
                rightEncoder.setPosition(0.0);
                leftEncoder.setPosition(0.0);
        }

        /** Passes through to the drivetrain to power the motors
         * @param forwardPower the drive forwards and backwards
         * @param turnPower the left/right rotation
         */
        public void arcadeDrive(Double forwardPower, Double turnPower) {
                driveTrain.arcadeDrive(forwardPower, turnPower);
        }

        /** a tank drive method */
        public void tankDrive(Double leftPower, Double rightPower) {
                driveTrain.tankDrive(leftPower, rightPower);
        }

        /** shifts the transmission into the specified gear
         * @param t true = high, false = low
         */
        public void setHighGear(Boolean t) {
                if (!t) {
                        shifter.set(DoubleSolenoid.Value.kReverse);
                        leftEncoder.setPositionConversionFactor(kMPR);
                        rightEncoder.setPositionConversionFactor(kMPR);
                }
                else {
                        shifter.set(DoubleSolenoid.Value.kForward);
                        leftEncoder.setPositionConversionFactor(kMPRH);
                        rightEncoder.setPositionConversionFactor(kMPRH);
                }
        }

        /** get the position from the odometery position
         * @return the position in encoder units
         */
        public Pose2d getPose() {
                return odometry.getPoseMeters();
        }

        /** resets the odometry system, called in {@link frc.robot.Robot} during autonomousInit() */
        public void resetOdometry(Pose2d pose) {
                resetEncoders();
                odometry.resetPosition(pose, Rotation2d.fromDegrees(getRawHeading()));
        }

        /** get the heading from the pidgeon IMU
         * @return the raw heading from the imu in degrees
         */
        public double getRawHeading() {
                double y = -ypr[0];
                while (y < 0)
                        y += 360;
                while (y > 360)
                        y -= 360;
                return y;
        }

        /** calculates the difference between the input and the actual robot heading
         * @param sp the target heading (degrees)
         * @return the difference between sp and the current imu heading (degrees)
         */
        public double getHeadingError(double sp) {
                double v = sp - getPose().getRotation().getDegrees();
                while (v < -180)
                        v += 360;
                while (v > 180)
                        v -= 360;
                return v;
        }

        /** updates the local imu rotation values */
        private void updateIMU() {
                _pigeon.getGeneralStatus(genStatus);
                _pigeon.getYawPitchRoll(ypr);
        }

        /** updates the position of the robot according to the IMU heading and the encoder positions */
        private void updateOdometry() {
                odometry.update(Rotation2d.fromDegrees(getRawHeading()), leftEncoder.getPosition(),
                                rightEncoder.getPosition());
        }

        /** returns roughly the speed at which the robot is rotating in degrees per second (kind of, not really a unit, just a value) */
        public double getRotationSpeed() {
                return Math.abs(lastYaw - ypr[0])*50;
        }
}
