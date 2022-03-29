package frc.robot.commands;

import frc.robot.subsystems.BigIronSubsystem;
import frc.robot.subsystems.KenobiSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import static frc.robot.Utils.Flags.*;

import static frc.robot.Constants.ControllerConstants.*;

/** Driver TeleOp Command 
 * @author Carl C.
*/
public class M_TeleOp extends CommandBase { // if M_TeleOp has a red line under it, check that the file is named M_TeleOp.java, the O keeps going undercase idk man it's strange
  private final BigIronSubsystem _bss;
  private final KenobiSubsystem _kss;
  private final Joystick _controller;
  private boolean climbActive = false;
  private boolean sentUp = false;

  /**
   * Manipulator TeleOp Command
   * @param BSS the shooting system
   * @param kss the climbing system
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
    sentUp = true; // I thought this was set false at init? idk why it works if this is true
    _bss.intakeDo(!_bss.intakeOut);
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    // Control Logic
    if(_controller.getRawButtonPressed(kA)) _bss.drumIdle = !_bss.drumIdle; // toggle drum idle
    _bss.runIntake(_controller.getRawAxis(kRightTrigger) > 0.3); // turn intake on/off (Deprecated, does nothing)
    _bss.intakeDo(_controller.getRawButtonPressed(kRightBumper)); // deploy/retract/release intake
    if (_controller.getRawButtonPressed(kLeftJoystickPressed)) _bss.ejectBall = !_bss.ejectBall; // toggle eject
    if (_controller.getRawButtonReleased(kLeftBumper)) { // when firing is finished
      _bss.ballCount = 1;
      _bss.ballFailedDebug();
      _bss.drumIdle = false;
    }
    if (_controller.getRawButtonPressed(kY)) _bss.ballFailedDebug(); // in case the robot's state gets messed up, manipulator can use the bonk button
    hoopTargeted = _controller.getRawButton(kLeftBumper); // begin targeting if lb is pressed
    _bss.fireTheBigIron = hoopTargeted;

    if (hoopTargeted) _bss.setAimDistance(targetDistance);

    // elevator logic
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