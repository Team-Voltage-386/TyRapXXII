// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.*;
import frc.robot.Constants;
import frc.robot.subsystems.BigIronSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.commands.BigIronIdle;
import frc.robot.commands.D_TeleOp;
import frc.robot.commands.M_TeleOp;
import frc.robot.commands.ShootBall;
import frc.robot.commands.ShootBallMan;
import frc.robot.commands.getBall;
import frc.robot.commands.D.Delay;
import frc.robot.commands.drive.LinearDrive;
import frc.robot.commands.drive.StationaryTurn;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final Joystick driverController = new Joystick(0);
  public static final Joystick manipulatorController = new Joystick(1);

  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem driveSubSystem = new DriveSubsystem();
  public final BigIronSubsystem bigIron = new BigIronSubsystem();
  public final LimeLightSubsystem LLSubsystem = new LimeLightSubsystem("limelight-xxii", Constants.LimeLightConstants.targetHeight, Constants.LimeLightConstants.mountAngle, Constants.LimeLightConstants.mountHeight, 0);

  // Shuffleboard declarations
  public static ShuffleboardTab driverTab;

  private final D_TeleOp driveTeleOp = new D_TeleOp(driveSubSystem, LLSubsystem);
  private final M_TeleOp manipTeleOp = new M_TeleOp(bigIron);

  // The robot's subsystems and commands are defined here...

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveSubSystem.setDefaultCommand(driveTeleOp);
    bigIron.setDefaultCommand(manipTeleOp);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    /*
    return new SequentialCommandGroup(
        new ParallelCommandGroup(new LinearDrive(driveSubSystem, 0.8, 0, false),new SequentialCommandGroup(new getBall(bigIron), new getBall(bigIron))),
        new StationaryTurn(driveSubSystem, 170, false),
        new ShootBall(bigIron,driveSubSystem,170.0,1),
        new ShootBall(bigIron,driveSubSystem,170.0,0),
        new StationaryTurn(driveSubSystem, 15, false),
        new ParallelCommandGroup(new LinearDrive(driveSubSystem, 1.55, 15, false),new getBall(bigIron)),
        new StationaryTurn(driveSubSystem, 180, false),
        new ShootBall(bigIron, driveSubSystem, 180,0)
        );
*/
 //standard
 /*
    return new SequentialCommandGroup(
      new ParallelCommandGroup(new LinearDrive(driveSubSystem, 1.5, 0, false),new SequentialCommandGroup(new getBall(bigIron),new getBall(bigIron))),
      new StationaryTurn(driveSubSystem, 170, false),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem)
    );*/

    /*
    return new ParallelCommandGroup(
      new SequentialCommandGroup(
        new LinearDrive(driveSubSystem, 2, 0, false),
        new StationaryTurn(driveSubSystem, 180, false),
        new LinearDrive(driveSubSystem, 2, 180, false),
        new StationaryTurn(driveSubSystem, 0, false)
      ),
      new BigIronIdle(bigIron)
    );*/

    return new SequentialCommandGroup(
      new Delay(2),
      new getBall(bigIron),
      new ShootBallMan(bigIron, driveSubSystem, LLSubsystem, 3150, 0.035)
    );
  }

  public Command getManCommand() {
    return null;
  }
}
