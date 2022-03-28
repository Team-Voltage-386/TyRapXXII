package frc.robot.commands;

import frc.robot.subsystems.BigIronSubsystem;
import frc.robot.subsystems.KenobiSubsystem;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Utils.Flags;

import static frc.robot.Utils.Flags.*;

import static frc.robot.Constants.ControllerConstants.*;

/** Driver TeleOp Command */
public class M_TeleOp extends CommandBase {
  private final BigIronSubsystem _bss;
  private final KenobiSubsystem _kss;
  private final Joystick _controller;
  private boolean climbActive = false;
  private boolean sentUp = false;

  /**
   * Manipulator TeleOp Command
   * @param BSS the BigIron
   */
  public M_TeleOp(BigIronSubsystem BSS, KenobiSubsystem kss) {
    _bss = BSS;
    _kss = kss;
    _controller = RobotContainer.manipulatorController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_bss);
    addRequirements(_kss);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    //_bss.reset();
    climbActive = false;
    _bss.drumIdle = false;
    sentUp = true;
    _bss.intakeDo(!_bss.intakeOut);
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    if(_controller.getRawButtonPressed(kA)) _bss.drumIdle = !_bss.drumIdle;
    _bss.runIntake(_controller.getRawAxis(kRightTrigger) > 0.3);
    _bss.intakeDo(_controller.getRawButtonPressed(kRightBumper));
    if (_controller.getRawButtonPressed(kLeftJoystickPressed)) _bss.ejectBall = !_bss.ejectBall;
    if (_controller.getRawButtonReleased(kLeftBumper)) {
      _bss.ballCount = 1;
      _bss.ballFailedDebug();
      _bss.drumIdle = false;
    }
    if (_controller.getRawButtonPressed(kY)) _bss.ballFailedDebug();
    hoopTargeted = _controller.getRawButton(kLeftBumper);
    _bss.fireTheBigIron = hoopTargeted;

    if (hoopTargeted) _bss.setAimDistance(targetDistance);

    if (climbActive) {
      if (sentUp) _kss.setElePower(-0.85* Math.pow(_controller.getRawAxis(kRightVertical),3));
      else {
        if (!(_kss.getEnc().getPosition() > 35)) _kss.setElePower(0.8);
        else {
          sentUp = true;
          _kss.setElePower(0);
        }
      }
    } else {
      _kss.setElePower(0);
      sentUp = false;
    }

    if (_controller.getRawButtonPressed(kX)) climbActive = !climbActive;
    if (_controller.getRawButtonPressed(kRightJoystickPressed)) _bss.reset();
    _bss.climbing = climbActive;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _bss.intakeDo(_bss.intakeOut);
  }
}