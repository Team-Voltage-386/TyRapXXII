
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BigIronSubsystem;
import static frc.robot.Constants.ControllerConstants.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ThreadedBall extends CommandBase {
  Thread ballThread;
  private final BigIronSubsystem _bss;
  private final Joystick _manipulatorController = RobotContainer.manipulatorController;
  // values needed in thread
  protected boolean waitingForAim;
  protected ArrayList balls;// 0-intake 1-mid 2-breach 3-shot

  /** Creates a new ThreadedBall. */
  public ThreadedBall(BigIronSubsystem BSS) {
    _bss = BSS;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(BSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballThread = new Thread(
        () -> {
          balls = new ArrayList<String>();
          for (int i = 0; i <= 3; i++) {
            balls.add("null");
          }
          while (!Thread.interrupted()) {
            _bss.getName();
            if (waitingForAim) {

            }
          }
        });
    ballThread.setDaemon(true);
    ballThread.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballThread.interrupt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
// when bringing balls to shooter
  private void shiftForward() {
    balls.add(0, "null");
    balls.remove(4);
  }
//when vomiting balls
  private void shiftBack() {
    balls.add(4, "null");
    balls.remove(0);
  }
}
