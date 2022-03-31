package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.Utils.Flags;
import frc.robot.subsystems.BigIronSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

import static frc.robot.Constants.DriveConstants.*;

/**
 * Shoots a ball at manually specified settings
 * @author Carl C.
 */
public class ShootBallMan extends CommandBase {

    private final BigIronSubsystem _bss;
    private final DriveSubsystem _dss;
    private final LimeLightSubsystem _lls;
    private final PIDController pidt = new PIDController(ltP,ltI,ltD);
    private int iBallCount = 0;
    private int ballShot = 0;
    boolean loaded = false;
    private final Timer tim = new Timer();

    private final int drumSpeed;
    private final double hoodPos;

    
    /** Creates a new ShootBallMan instruction
     * @param angle angle set
     * @param rel if true, angle set is relative
    */
    public ShootBallMan(BigIronSubsystem BSS,DriveSubsystem DSS, LimeLightSubsystem LLS,int ds, double hp) {
        _bss = BSS;
        _dss = DSS;
        _lls = LLS;
        drumSpeed = ds;
        hoodPos = hp;
        addRequirements(_dss);
        addRequirements(_bss);
    }

    @Override
    public void initialize() {
        tim.stop();
        tim.reset();
        iBallCount = _bss.ballCount;
        _bss.fireTheBigIron = true;
        _lls.lights(true);
        loaded = false;
        ballShot = 0;
        _bss.pidD.reset();
    } 

    @Override
    public void execute() {
        _bss.fireTheBigIron = true;
        Utils.Flags.targetDistance = _lls.metersToTarget();

        _bss.drumSP = drumSpeed; // not efficient but doesn't matter because it's just for testing. All it needs to do it make sure the state is proper
        _bss.hoodSet = hoodPos;

        Flags.hoopLocked = !(Math.abs(_lls.tx) > 2);

        // logic for when to end
        if (!loaded && _bss.breachSensorFlag) {
            loaded = true;
        } else if (loaded && !_bss.breachSensorFlag) {
            loaded = false;
            ballShot++;
        }
        _dss.arcadeDrive(0.0, pidt.calculate(_lls.tx)); // aim
    }

    @Override
    public void end(boolean interuppted) {
        _bss.fireTheBigIron = false; // clean up
        _bss.ballCount = 0;
    }

    @Override
    public boolean isFinished() {
        return ballShot == iBallCount; // shoot all the balls
    }
}