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
public class TeleOp_M extends CommandBase { // if M_TeleOp has a red line under it, check that the file is named M_TeleOp.java, the O keeps going undercase idk man it's strange
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
  public TeleOp_M(BigIronSubsystem BSS, KenobiSubsystem kss) {
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
    //_bss.intakeUpdate(!_bss.intakeOut);
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    // Control Logic
    if(_controller.getRawButtonPressed(kA)) _bss.drumIdle = !_bss.drumIdle; // toggle drum idle
    _bss.intakeUpdate(_controller.getRawButtonPressed(kRightBumper)); // deploy/retract/release intake
    if (triggerPressed(kLeftTrigger)) _bss.lowShot = !_bss.lowShot; // toggle eject
    if (triggerReleased(kRightTrigger)) { // when firing is finished
      _bss.ballCount = 1;
      _bss.decreaseBC();
      _bss.drumIdle = false;
    }
    hoopTargeted = getTriggerController(kRightTrigger); // begin targeting if rt is pressed
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

    if (_controller.getRawButtonPressed(kB)) _bss.decreaseBC();
    if (_controller.getRawButtonPressed(kY)) _bss.increaseBC();

    if (_controller.getRawButtonPressed(kX)) climbActive = !climbActive;
    if (_controller.getRawButtonPressed(kRightJoystickPressed)) _bss.reset();
    _bss.climbing = climbActive;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _bss.intakeUpdate(_bss.intakeOut);
  }

  private boolean lastCycleTrigger = false;
  private boolean triggerReleased(int button) {
    boolean cont = getTriggerController(button);
    boolean res = !cont;
    res = res && lastCycleTrigger;
    lastCycleTrigger = cont;
    return res;
  }

  private boolean[] lastCont = {false,false};
  private boolean getTriggerController(int button) {
    boolean res = false;
    if ((_controller.getRawAxis(button) > 0.6) && !lastCont[button-2]) res = true;
    else if ((_controller.getRawAxis(button) < 0.4)&&lastCont[button-2]) res = false;
    else res = lastCont[button-2];
    lastCont[button-2] = res;
    return res;
  }

  private boolean lastCycleTriggerPressed = false;
  private boolean triggerPressed(int button) {
    boolean cont = getTriggerController(button);
    boolean res = cont;
    res = res && !lastCycleTriggerPressed;
    lastCycleTriggerPressed = cont;
    return res;
  }
}