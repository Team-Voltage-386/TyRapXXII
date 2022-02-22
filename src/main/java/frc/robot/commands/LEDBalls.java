// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.plaf.basic.BasicLabelUI;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BigIronSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj.util.Color;

public class LEDBalls extends CommandBase {
  private final BigIronSubsystem _bss;
  private final LEDSubsystem _lss;

  /** Creates a new LEDBalls. */
  public LEDBalls(BigIronSubsystem BSS, LEDSubsystem LSS) {
    _bss = BSS;
    _lss = LSS;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(BSS);
    addRequirements(LSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _lss.startLED();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_bss.ball1Col.equals("Blue")) {
      _lss.setLEDSegment(8, 15, Color.kBlue);
    } else if (_bss.ball1Col.equals("Red")) {
      _lss.setLEDSegment(8, 15, Color.kRed);
    } else {
      _lss.setLEDSegmentOff(8, 15);
    }

    if (_bss.ball2Col.equals("Blue")) {
      _lss.setLEDSegment(0, 7, Color.kBlue);
    } else if (_bss.ball2Col.equals("Red")) {
      _lss.setLEDSegment(0, 7, Color.kRed);
    } else {
      _lss.setLEDSegmentOff(0, 7);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _lss.stopLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
