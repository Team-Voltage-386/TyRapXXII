package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ExternalFollower;

import static frc.robot.Constants.DriveConstants.*;

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
        public final DifferentialDrive driveTrain = new DifferentialDrive(rearLeftMotor, frontRightMotor);

        // Sensor instantiations
        RelativeEncoder leftEncoder = rearLeftMotor.getEncoder();
        RelativeEncoder rightEncoder = frontRightMotor.getEncoder();

        // Creates a shuffleboard tab for the drive
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

                rearLeftMotor.setInverted(true);
                frontRightMotor.setInverted(false);
                frontLeftMotor.follow(rearLeftMotor);// front left yields faulty encoder values so that set follower
                rearRightMotor.follow(frontRightMotor);

        }

        @Override
        public void periodic() {
                 //This method will be called once per scheduler run
                
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

        // arcade drive method to be called by commands
        public void arcadeDrive(Double forwardPower, Double turnPower) {
                driveTrain.arcadeDrive(forwardPower, turnPower);
        }

        //tank drive method to be called by commands
        public void tankDrive(Double leftPower, Double rightPower) {
                driveTrain.tankDrive(leftPower, rightPower);
        }

        @Override
        public void simulationPeriodic() {
                // This method will be called once per scheduler run during simulation
        }
}
