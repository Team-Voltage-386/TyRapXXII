// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.*;
import frc.robot.Constants;
import frc.robot.subsystems.BigIronSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.KenobiSubsystem;
import frc.robot.commands.D_TeleOp;
import frc.robot.commands.Delay;
import frc.robot.commands.M_TeleOp;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.commands.BigIronIdle;
import frc.robot.commands.ShootBall;
import frc.robot.commands.ShootBallMan;
import frc.robot.commands.getBall;
import frc.robot.commands.drive.LinearDrive;
import frc.robot.commands.drive.StationaryTurn;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static frc.robot.Constants.BigIronConstants.*;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Utils;

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
  private final KenobiSubsystem kenobi = new KenobiSubsystem();
  public final LimeLightSubsystem LLSubsystem = new LimeLightSubsystem("limelight-xxii", Constants.LimeLightConstants.targetHeight, Constants.LimeLightConstants.mountAngle, Constants.LimeLightConstants.mountHeight, 0);

  // Shuffleboard declarations
  public static ShuffleboardTab driverTab;

  private final D_TeleOp driveTeleOp = new D_TeleOp(driveSubSystem, LLSubsystem);
  private final M_TeleOp manipTeleOp = new M_TeleOp(bigIron, kenobi);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final AutoRoutines autos = this.new AutoRoutines();

  private static final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");

  // The robot's subsystems and commands are defined here...

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    Utils.ourAlliance = DriverStation.getAlliance().toString();
    Utils.antiAlliance = Utils.giveAntiAlliance(Utils.ourAlliance);
    driveSubSystem.setDefaultCommand(driveTeleOp);
    bigIron.setDefaultCommand(manipTeleOp);

    autoChooser.setDefaultOption("Basic 2 Ball", autos.basiciiBall);
    autoChooser.addOption("iiiBallA", autos.iiiBallA);
    autoChooser.addOption("vBall", autos.vBall);
    autoChooser.addOption("TuningTest", autos.tuningTest);
    autoChooser.addOption("ShooterTest", autos.shootTest);
    mainTab.add("AutoRoutine",autoChooser);
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
    return autoChooser.getSelected();
  }

  public Command getManCommand() {
    return null;
  }

  private final class AutoRoutines {
    public final Command basiciiBall = new SequentialCommandGroup(
      new ParallelCommandGroup(
        new LinearDrive(driveSubSystem, 1.5, 0, false,1),
        new SequentialCommandGroup(new getBall(bigIron),
        new getBall(bigIron))),
      new StationaryTurn(driveSubSystem, 180, false),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem)
    );
    public final Command iiiBallA = new ParallelCommandGroup(new SequentialCommandGroup(
        new LinearDrive(driveSubSystem,1.5,0,false,1),
        new StationaryTurn(driveSubSystem, 170, false),
        new StationaryTurn(driveSubSystem, 10, false),
        new LinearDrive(driveSubSystem,3.15, 8,false,1),
        new LinearDrive(driveSubSystem, 2, 8, false, -1),
        new StationaryTurn(driveSubSystem, 170, false)
      ),
      new BigIronIdle(bigIron)
    );
    public final Command vBall = new ParallelCommandGroup(
      new SequentialCommandGroup(
        new LinearDrive(driveSubSystem, 1.2, 0, false, -1),
        new StationaryTurn(driveSubSystem, -80, false),
        new LinearDrive(driveSubSystem, 2, -90, false, 1),
        new StationaryTurn(driveSubSystem, -45, false),
        new LinearDrive(driveSubSystem, 2.5, -40, false, 1),
        new StationaryTurn(driveSubSystem, 80, false),
        new StationaryTurn(driveSubSystem, -120, false),
        new LinearDrive(driveSubSystem,3,-130,false,1),
        new LinearDrive(driveSubSystem,2,-130,false,-1),
        new StationaryTurn(driveSubSystem, 80, false)
      ),
      new BigIronIdle(bigIron)
    );
    public final Command tuningTest = new ParallelCommandGroup(
      new SequentialCommandGroup(
        new LinearDrive(driveSubSystem, 2, 0, false,1),
        new StationaryTurn(driveSubSystem, 180, false),
        new LinearDrive(driveSubSystem, 2, 180, false,1),
        new StationaryTurn(driveSubSystem, 0, false)
      ),
      new BigIronIdle(bigIron)
    );
    public final Command shootTest = new SequentialCommandGroup(
      new Delay(2),
      new getBall(bigIron),
      new ShootBallMan(bigIron, driveSubSystem, LLSubsystem, 3650, 0.35)
    );
  }
}
