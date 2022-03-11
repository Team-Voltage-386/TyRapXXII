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

    // Utils.ourAlliance = DriverStation.getAlliance().toString();
    // Utils.antiAlliance = Utils.giveAntiAlliance(Utils.ourAlliance);

    autoChooser.setDefaultOption("Basic 2 Ball", autos.basiciiBall);
    autoChooser.addOption("3 Ball A", autos.iiiBallA);
    autoChooser.addOption("4 Ball B", autos.ivBallB);
    //autoChooser.addOption("TuningTest", autos.tuningTest);
    //autoChooser.addOption("ShooterTest", autos.shootTest);
    autoChooser.addOption("MartianRock", autos.marRock);
    autoChooser.addOption("4 Ball A (HP)", autos.ivBallA);
    mainTab.add("AutoRoutine",autoChooser).withPosition(0,0).withSize(3,1);
    bigIron.setDefaultCommand(manipTeleOp);
    driveSubSystem.setDefaultCommand(driveTeleOp);
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
    return new ParallelCommandGroup(driveTeleOp,manipTeleOp);
  }

  private final class AutoRoutines {
    public final Command basiciiBall = new SequentialCommandGroup(
      new ParallelCommandGroup(
        new LinearDrive(driveSubSystem, 1.5, 0, false,1),
        new SequentialCommandGroup(
          new getBall(bigIron,3),
          new getBall(bigIron,3))
          ),
      new StationaryTurn(driveSubSystem, 180, false),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem)
    );
    public final Command iiiBallA =new SequentialCommandGroup(
      new ParallelCommandGroup(
        new LinearDrive(driveSubSystem,1.5,0,false,1),
        new SequentialCommandGroup(
          new getBall(bigIron,3),
          new getBall(bigIron,3)
        )
      ),
      new StationaryTurn(driveSubSystem, 160, false),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem),
      new StationaryTurn(driveSubSystem, 14, false),
      new ParallelCommandGroup(
        new LinearDrive(driveSubSystem,3.2, 12,false,1),
        new getBall(bigIron,4)
      ),
      new LinearDrive(driveSubSystem, 1, 8, false, -1),
      new StationaryTurn(driveSubSystem, 170, false),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem)
    );
    public final Command ivBallA = new SequentialCommandGroup(
      new ParallelCommandGroup(
        new LinearDrive(driveSubSystem,1.5,0,false,1),
        new SequentialCommandGroup(
          new getBall(bigIron,3),
          new getBall(bigIron,3)
        )
      ),
      new StationaryTurn(driveSubSystem, 160, false),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem),
      new StationaryTurn(driveSubSystem, 14, false),
      new ParallelCommandGroup(
        new LinearDrive(driveSubSystem,3.2, 12,false,1),
        new getBall(bigIron,4)
      ),
      new getBall(bigIron,0.8),
      new LinearDrive(driveSubSystem, 1, 8, false, -1),
      new StationaryTurn(driveSubSystem, 170, false),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem)
    );
    public final Command ivBallB = new SequentialCommandGroup(
      new ParallelCommandGroup(
        new LinearDrive(driveSubSystem, 1.25, 0, false, -1),
        new getBall(bigIron,2)
      ),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem),
      new StationaryTurn(driveSubSystem, -100, false),
      new ParallelCommandGroup(
        new LinearDrive(driveSubSystem, 2, -105, false, 1),
        new getBall(bigIron,2.5)
      ),
      new StationaryTurn(driveSubSystem, -45, false),
      new ParallelCommandGroup(
        new LinearDrive(driveSubSystem, 2.5, -40, false, 1),
        new getBall(bigIron,3.5)
      ),
      new StationaryTurn(driveSubSystem, 50, false),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem),
      new StationaryTurn(driveSubSystem, -110, false),
      new ParallelCommandGroup(
        new LinearDrive(driveSubSystem,3,-124,false,1),
        new getBall(bigIron,4)
      ),
      new LinearDrive(driveSubSystem,1,-130,false,-1),
      new StationaryTurn(driveSubSystem, 50, false),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem)
    );
    public final Command marRock = new SequentialCommandGroup(
      new Delay(10),
      new LinearDrive(driveSubSystem, 1.4, 0, false, 1)
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
      new getBall(bigIron,2),
      new ShootBallMan(bigIron, driveSubSystem, LLSubsystem, 1500, 0.35)
    );
  }
}
