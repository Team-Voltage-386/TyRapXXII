// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.subsystems.BallMovementSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


import static frc.robot.Constants.ControllerConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/** An example command that uses an example subsystem. */
public class ManualArcadeDriveCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem aSubsystem;
  private final BallMovementSubsystem _bmss;
  private boolean shiftUp = false;
 

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualArcadeDriveCommand(DriveSubsystem subsystem, BallMovementSubsystem bmss) {
    aSubsystem = subsystem;
    _bmss = bmss;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(bmss);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    aSubsystem.setHighGear(false);
    rootForward = 0;
    rootTurn = 0;
  }

  private double rootForward, rootTurn;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _bmss.runIntake(RobotContainer.driverController.getRawButton(kLeftBumper));
    rootForward = RobotContainer.driverController.getRawAxis(kLeftVertical);
    rootTurn = -1 * RobotContainer.driverController.getRawAxis(kRightHorizontal);
    aSubsystem.arcadeDrive(rootForward, rootTurn);
    if (RobotContainer.driverController.getRawButtonPressed(kA)) {
      shiftUp = !shiftUp;
      aSubsystem.setHighGear(shiftUp);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
