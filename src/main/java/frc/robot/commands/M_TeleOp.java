package frc.robot.commands;

import frc.robot.subsystems.BigIronSubsystem;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import static frc.robot.Utils.Flags.*;

import static frc.robot.Constants.ControllerConstants.*;

/** Driver TeleOp Command */
public class M_TeleOp extends CommandBase {
  private final BigIronSubsystem _bss;
  private final Joystick _controller;

  /**
   * Manipulator TeleOp Command
   * @param BSS the BigIron
   */
  public M_TeleOp(BigIronSubsystem BSS) {
    _bss = BSS;
    _controller = RobotContainer.manipulatorController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_bss);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    _bss.reset();
    //_bss.intakeDo(_bss.intakeOut);
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    if(_controller.getRawButtonPressed(kX)) _bss.drumIdle = !_bss.drumIdle;
    _bss.runIntake(_controller.getRawAxis(kRightTrigger) > 0.3);
    _bss.intakeDo(_controller.getRawButtonPressed(kRightBumper));
    if (_controller.getRawButtonPressed(kB)) _bss.ejectBall = !_bss.ejectBall;

    if (_controller.getRawButtonPressed(kY)) _bss.ballFailedDebug();

    hoopTargeted = _controller.getRawButton(kLeftBumper);
    _bss.fireTheBigIron = hoopTargeted;
    if (hoopTargeted) _bss.setAimDistance(targetDistance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _bss.intakeDo(_bss.intakeOut);
  }
}