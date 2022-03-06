package frc.robot.commands;

import frc.robot.subsystems.BigIronSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Utils.Flags.*;

import com.fasterxml.jackson.core.TreeCodec;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Utils;
import frc.robot.Utils.Flags;

import static frc.robot.Constants.ControllerConstants.*;

/** Driver TeleOp Command */
public class D_TeleOp extends CommandBase {
  private final DriveSubsystem _dss;
  private final Joystick _controller;
  private final LimeLightSubsystem _lls;
  private final PIDController pid = new PIDController(ltP, ltI, ltD);
  private double rootTurn;
  public Boolean ballFound = false;
  private boolean highGear = false;
  private double rootDrive = 0;

  /**
   * Driver TeleOp Command
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
    pid.setTolerance(1, 1);
    //_lls.setPipeLine(0);
    //_lls.lights(true);
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    Flags.hoopVisible = _lls.targetFound;
    Flags.complianceOverride = _controller.getRawButton(kRightBumper);
    double controllerIn = _controller.getRawAxis(kLeftVertical);
    if (Math.abs(controllerIn) > Math.abs(rootDrive)) rootDrive = Utils.lerp(rootDrive, controllerIn, kSmoothingAccelFactor);
    else rootDrive = Utils.lerp(rootDrive, controllerIn, kSmoothingDecelFactor);
    rootTurn = -_controller.getRawAxis(kRightHorizontal);
    if (_controller.getRawButtonPressed(kLeftBumper)) {
      if (!(highGear && Math.abs(rootDrive) > kMaxDownshiftPower)) {
        highGear = !highGear;
        _dss.setHighGear(highGear);
      }
    }
    if (_lls.targetFound && hoopTargeted) {
      //_controller.setRumble(RumbleType.kRightRumble, 0.5);
      if (Math.abs(_lls.tx) > 1.7) {
        hoopLocked = false;
        rootTurn += MathUtil.clamp(pid.calculate(_lls.tx), -tC, tC);
      }
      else {
        pid.reset();
        hoopLocked = true;
        rootTurn = 0;
      }
    } 
    else {
      pid.reset();
    }
    _dss.arcadeDrive(rootDrive, rootTurn);
    if (_lls.targetFound) targetDistance = _lls.metersToTarget();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //_lls.lights(false);
  }


  
}