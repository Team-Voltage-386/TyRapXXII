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
import frc.robot.subsystems.BigIronSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.KenobiSubsystem;
import frc.robot.commands.TeleOp_D;
import frc.robot.commands.Delay;
import frc.robot.commands.TeleOp_M;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.commands.BigIronIdle;
import frc.robot.commands.ShootBall;
import frc.robot.commands.ShootBallMan;
import frc.robot.commands.GetBall;
import frc.robot.commands.drive.LinearDrive;
import frc.robot.commands.drive.LinearDriveHigh;
import frc.robot.commands.drive.StationaryTurn;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
  public final LimeLightSubsystem LLSubsystem = new LimeLightSubsystem("limelight", 0);

  // Shuffleboard declarations
  public static ShuffleboardTab driverTab;

  private final TeleOp_D driveTeleOp = new TeleOp_D(driveSubSystem, LLSubsystem);
  private final TeleOp_M manipTeleOp = new TeleOp_M(bigIron, kenobi);
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

    autoChooser.setDefaultOption("Basic 2 Ball", autos.basiciiBall);
    autoChooser.addOption("High Auto", autos.highAuto);
    autoChooser.addOption("V-Ball", autos.vBall);
    autoChooser.addOption("TuningTest", autos.tuningTest);
    autoChooser.addOption("TuningTestSquare", autos.tuningTestII);
    autoChooser.addOption("ShooterTest", autos.shootTest);
    autoChooser.addOption("MartianRock", autos.marRock);
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
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**@return the two teleop commands in parallel*/
  public Command getManCommand() {
    return new ParallelCommandGroup(driveTeleOp,manipTeleOp);
  }

  /** All the auto routines are in this class for organization
   * @author Carl C.
   */
  private final class AutoRoutines {
    /** drives off the tarmac, grabs the ball directly in front, and fires both.<p> As long as the shooter is calibrated this is fantastically reliable */
    public final Command basiciiBall = new SequentialCommandGroup(
      new LinearDrive(driveSubSystem, 1.5, 0, false,1),
      new StationaryTurn(driveSubSystem, 180, false),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem, 2)
    );
    /** A 4 ball auto from the primary starting position <p> needs human player interaction<p> Works Great */
    public final Command highAuto = new SequentialCommandGroup(
      new LinearDrive(driveSubSystem,1.5,0,false,1),
      new StationaryTurn(driveSubSystem, 170, false),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem, 2.6),
      new StationaryTurn(driveSubSystem, 14, false),
      new LinearDrive(driveSubSystem,3.24, 12,false,1),
      new LinearDrive(driveSubSystem, 2, 8, false, -1),
      new StationaryTurn(driveSubSystem, 170, false),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem, 2.6)
    );

    /** 4 ball from the starting position opposite the hangar, grabs all but the hangar ball and the human player's ball, 
     * not functioning yet
    */
    public final Command vBall = new SequentialCommandGroup(
      new LinearDrive(driveSubSystem, 1.25, 0, false, -1),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem, 2),
      new StationaryTurn(driveSubSystem, -90, false),
      new LinearDrive(driveSubSystem, 2, -90, false, 0.8),
      new StationaryTurn(driveSubSystem, -45, false),
      new LinearDrive(driveSubSystem, 2.5, -40, false, 1),
      new StationaryTurn(driveSubSystem, 50, false),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem,2.6),
      new StationaryTurn(driveSubSystem, -95, false),
      new LinearDrive(driveSubSystem,3,-100,false,1),
      new LinearDrive(driveSubSystem,1,-100,false,-1),
      new StationaryTurn(driveSubSystem, 50, false),
      new ShootBall(bigIron, driveSubSystem, LLSubsystem, 2.6)
    );
    /** waits 10 seconds and drives out of the tarmac */
    public final Command marRock = new SequentialCommandGroup(
      new Delay(10),
      new LinearDrive(driveSubSystem, 1.4, 0, false, 1)
    );
    /** the robot drives forward, turns 180, drives back, then 180 again, used to test driving tuning */
    public final Command tuningTest = new ParallelCommandGroup(
      new SequentialCommandGroup(
        new LinearDrive(driveSubSystem, 2, 0, false,1),
        new StationaryTurn(driveSubSystem, 180, false),
        new LinearDrive(driveSubSystem, 2, 180, false,1),
        new StationaryTurn(driveSubSystem, 0, false)
      ),
      new BigIronIdle(bigIron)
    );
    public final Command tuningTestII = new ParallelCommandGroup(
      new SequentialCommandGroup(
        new LinearDrive(driveSubSystem, 1.5, 0, false, 1),
        new StationaryTurn(driveSubSystem, 90, false),
        new LinearDrive(driveSubSystem, 1.5, 90, false, 1),
        new StationaryTurn(driveSubSystem, 180, false),
        new LinearDrive(driveSubSystem, 1.5, 180, false, 1),
        new StationaryTurn(driveSubSystem, -90, false),
        new LinearDrive(driveSubSystem, 1.5, -90, false, 1),
        new StationaryTurn(driveSubSystem, 0, false)
      ),
      new BigIronIdle(bigIron)
    );
    /** simply fires, change the values in the shootballman instruction to test settings */
    public final Command shootTest = new SequentialCommandGroup(
      new Delay(2),
      new GetBall(bigIron,2),
      new ShootBallMan(bigIron, driveSubSystem, LLSubsystem, 1500, 0.35)
    );
  }
}
