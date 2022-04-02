package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Utils.Flags.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Utils;
import frc.robot.Utils.Flags;

import static frc.robot.Constants.ControllerConstants.*;

/** Driver TeleOp Command 
 * @author Carl C.
*/
public class D_TeleOp extends CommandBase {
  private final DriveSubsystem _dss;
  private final Joystick _controller;
  private final LimeLightSubsystem _lls;
  private double rootTurn;
  public Boolean ballFound = false;
  private boolean highGear = false;
  private double rootDrive = 0;

/**
 * Creates a Driver TeleOp command
 * 
 * @param DSS  The drive subsystem used by this command.
 * @param BSS the BigIron
 * @param LLS  the hoop LL subsystem used by this command.
 */
  public D_TeleOp(DriveSubsystem DSS, LimeLightSubsystem LLS) {
    _dss = DSS;
    _lls = LLS;
    _controller = RobotContainer.driverController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_dss);
    addRequirements(_lls);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    rootDrive = 0;
    rootTurn = 0;
    ltPID.setTolerance(1, 1);
    _dss.setHighGear(false);
    //_lls.setPipeLine(0);
    //_lls.lights(true);
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    //set flags
    Flags.hoopVisible = _lls.targetFound;
    Flags.complianceOverride = _controller.getRawButton(kRightBumper);

    //driving logic, uses lerpA to smooth the drive 
    double controllerIn = _controller.getRawAxis(kLeftVertical);
    if (Math.abs(controllerIn) > Math.abs(rootDrive)) rootDrive = Utils.lerpA(rootDrive, controllerIn, kSmoothingAccelFactor);
    else rootDrive = Utils.lerpA(rootDrive, controllerIn, kSmoothingDecelFactor);
    rootTurn = -_controller.getRawAxis(kRightHorizontal);

    // Change gear on left bumper
    if (_controller.getRawButtonPressed(kLeftBumper)) {
      highGear = !highGear;
      _dss.setHighGear(highGear);
    }

    // logic for the alignment of the robot
    if (_lls.targetFound && hoopTargeted) {
      //_controller.setRumble(RumbleType.kRightRumble, 0.5);
      if (Math.abs(_lls.tx) > 1.2) {
        hoopLocked = false;
        rootTurn = ltALG.get(_lls.tx);
      }
      else {
        ltPID.reset();
        hoopLocked = true;
        rootTurn = 0;
      }
    } 
    else {
      ltPID.reset();
    }

    // set drive and distance flag
    _dss.arcadeDrive(rootDrive, rootTurn);
    if (_lls.targetFound) targetDistance = _lls.metersToTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //_lls.lights(false);
  } 
}