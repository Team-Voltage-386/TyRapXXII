// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.BigIronConstants;
import frc.robot.subsystems.BigIronSubsystem;
import frc.robot.subsystems.KenobiSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

import static frc.robot.Constants.ControllerConstants.*;

public class M_Teleop extends CommandBase {
  private final KenobiSubsystem _kss;
  private final BigIronSubsystem _bss;
  private final Joystick _manipulatorController;

  /** Creates a new M_Teleop. */
  public M_Teleop(KenobiSubsystem KSS, BigIronSubsystem BSS) {
    _kss = KSS;
    _bss = BSS;
    _manipulatorController = RobotContainer.manipulatorController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_kss, _bss);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _kss.armsIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_manipulatorController.getRawButtonPressed(kX)) {
      _kss.armsDo();
    }

    _bss.intakeDo(_manipulatorController.getRawButtonPressed(kRightBumper));

    _kss.elevatorDo(_manipulatorController.getRawAxis(kRightVertical) * 0.5);
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
