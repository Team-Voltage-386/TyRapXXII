package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
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
    private final int drumS;
    private boolean done = false;
    boolean loaded = false;
    private final double hsp;
    private final Timer tim = new Timer();

    
    /**@param angle angle set
     * @param rel if true, angle set is relative
    */
    public ShootBall(BigIronSubsystem BSS,DriveSubsystem DSS, double angle, int drumSpeed,int finBallCount,double hood) {
        _bss = BSS;
        _dss = DSS;
        a = angle;
        fbc = finBallCount;
        drumS = drumSpeed;
        hsp = hood;
        addRequirements(_dss);
        addRequirements(_bss);
    }

    @Override
    public void initialize() {
        tim.stop();
        tim.reset();
        iBallCount = _bss.ballCount;
        _bss.ejectBall = true;
        _bss.drumSP = drumS;
        _bss.fireTheBigIron = true;
        loaded = false;
        _bss.hoodSet = hsp;
        _bss.pidD.reset();
    } 

    @Override
    public void execute() {
        if (_bss.ballCount == fbc) timer.start();
        _dss.arcadeDrive(0.0, pidt.calculate(_dss.getHeadingError(a)));
        if (!loaded && _bss.breachSensorFlag) {
            loaded = true;
            tim.start();
        }
        else if (loaded && !_bss.breachSensorFlag && tim.hasElapsed(0.2)) {
            _bss.fireTheBigIron = false;
            done = true;
        }
    }

    @Override
    public void end(boolean interuppted) {
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}