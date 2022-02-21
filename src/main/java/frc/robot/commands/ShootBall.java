package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BigIronSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.DriveConstants.*;

public class ShootBall extends CommandBase {

    private final BigIronSubsystem _bss;
    private final DriveSubsystem _dss;
    private final PIDController pidt = new PIDController(tP,tI,tD);
    private int iBallCount = 0;
    private final int fbc;
    private final double a;
    private final Timer timer = new Timer();
    
    /**@param angle angle set
     * @param rel if true, angle set is relative
    */
    public ShootBall(BigIronSubsystem BSS,DriveSubsystem DSS, double angle,int finBallCount) {
        _bss = BSS;
        _dss = DSS;
        a = angle;
        fbc = finBallCount;
        addRequirements(_dss);
        addRequirements(_bss);
    }

    @Override
    public void initialize() {
        iBallCount = _bss.ballCount;
        _bss.ejectBall = true;
    } 

    @Override
    public void execute() {
        if (_bss.ballCount == fbc) timer.start();
        _dss.arcadeDrive(0.0, pidt.calculate(_dss.getHeadingError(a)));
    }

    @Override
    public void end(boolean interuppted) {
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.15);
    }
}