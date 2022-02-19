package frc.robot.commands;


import frc.robot.subsystems.BigIronSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.DriveConstants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import static frc.robot.Constants.ControllerConstants.*;

/**Driver TeleOp Command*/
public class D_TeleOp extends CommandBase {
  private final DriveSubsystem _dss;
  private final BigIronSubsystem _bss;
  private final Joystick _controller;
  private final PIDController pid = new PIDController(tP, tI, tD);
  private double rootForward, rootTurn;
  public Boolean ballFound = false;
  

  /**Driver TeleOp Command
   * @param DSS The drive subsystem used by this command.
   * @param LLS the hoop LL subsystem used by this command.
   * @param LLSB the ball LL subsystem used by this command.
   */
  public D_TeleOp(DriveSubsystem DSS, BigIronSubsystem BSS) {
    _dss = DSS;
    _bss = BSS;
    _controller = RobotContainer.driverController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_dss);
  }

  /**Called when the command is initially scheduled.*/
  @Override
  public void initialize() {
    rootForward = 0;
    rootTurn = 0;
    pid.setTolerance(1,1);
    _bss.runHoodMan = true;
  }

  /**Called every time the scheduler runs while the command is scheduled.*/
  @Override
  public void execute() {
    rootForward = _controller.getRawAxis(kLeftVertical);
    rootTurn = _controller.getRawAxis(kRightHorizontal);

    _bss.hoodManPower = _controller.getRawAxis(kRightVertical);
    _bss.runBeltMan = _controller.getRawButton(kX);
    _dss.arcadeDrive(rootForward, rootTurn);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}