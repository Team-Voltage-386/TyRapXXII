package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Utils.Flags.*;

import frc.robot.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Utils;
import frc.robot.Utils.Flags;

import static frc.robot.Constants.ControllerConstants.*;

/** Driver TeleOp Command 
 * @author Carl C.
*/
public class TeleOp_D extends CommandBase {
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
  public TeleOp_D(DriveSubsystem DSS, LimeLightSubsystem LLS) {
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

  double integralTurnAdjust = 0;
  double lastTurn = 0;

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    //set flags
    Flags.hoopVisible = _lls.targetFound;
    Flags.complianceOverride = _controller.getRawButton(kRightBumper);

    //driving logic, uses lerpA to smooth the drive 
    double controllerIn = _controller.getRawAxis(kLeftVertical);

    if (_dss.highGear) {
      controllerIn *= highGearInputLimit;
      controllerIn -= (1-highGearInputLimit)*_controller.getRawAxis(kRightTrigger);
    }
    if (Math.abs(controllerIn) > Math.abs(rootDrive)) rootDrive = Utils.lerpA(rootDrive, controllerIn, kSmoothingAccelFactor);
    else rootDrive = Utils.lerpA(rootDrive, controllerIn, kSmoothingDecelFactor);

    //turn behavior, uses an integral of the stick derivitive to help prevent overshoot
    double contTurn = -_controller.getRawAxis(kRightHorizontal);
    integralTurnAdjust += contTurn - lastTurn;
    rootTurn = contTurn;
    if (_dss.highGear) rootTurn += (2*integralTurnAdjust);
    lastTurn = contTurn;
    if (_dss.highGear) rootTurn *= highGearTurnLimit;

    // Change gear on left bumper
    if (_controller.getRawButtonPressed(kLeftBumper)) highGear = !highGear;
    if (!hoopTargeted) _dss.setHighGear(highGear);
    else _dss.setHighGear(false);

    // logic for the alignment of the robot
    if (_lls.targetFound && hoopTargeted) {
      if (Math.abs(_lls.tx) > 1.2) {
        hoopLocked = false;
        rootTurn = ltALG.get(_lls.tx);
      }
      else {
        hoopLocked = true;
      }
      if (Math.abs(_lls.tx) > 5 || Math.abs(_lls.tx) < 0.4) ltPID.reset();
    } 
    else {
      ltPID.reset();
    }
    // set drive and distance flag
    _dss.arcadeDrive(rootDrive, rootTurn);
    if (_lls.targetFound) targetDistance = _lls.metersToTarget();

    integralTurnAdjust *= 0.9;

    Logger.setBoolean(1, hoopTargeted);
    Logger.setBoolean(2, hoopLocked);
    Logger.setDouble(3, targetDistance);
    Pose2d p = _dss.getPose();
    Logger.setDouble(4, p.getRotation().getDegrees());
    Logger.setDouble(5, p.getX());
    Logger.setDouble(6, p.getY());
    Logger.setDouble(8, _lls.tx);
    Logger.setDouble(9, rootDrive);
    Logger.setDouble(12, rootTurn);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //_lls.lights(false);
  } 
  
}