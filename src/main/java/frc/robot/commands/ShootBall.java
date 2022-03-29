package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.Utils.Flags;
import frc.robot.subsystems.BigIronSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

import static frc.robot.Constants.DriveConstants.*;

/**Shoots all the balls in the robot
 * @author Carl C.
 */
public class ShootBall extends CommandBase {

    private final BigIronSubsystem _bss;
    private final DriveSubsystem _dss;
    private final LimeLightSubsystem _lls;
    private final PIDController pidt = new PIDController(ltP,ltI,ltD);
    private int iBallCount = 0;
    private int ballShot = 0;
    private final Timer timer = new Timer();
    private boolean done = false;
    boolean loaded = false;
    private final Timer tim = new Timer();

    
    /**Creates a new ShootBall instruction
     * @param BSS the bigIron
     * @param DSS the drivesubsystem
     * @param LLS the limelight used for targeting
    */
    public ShootBall(BigIronSubsystem BSS,DriveSubsystem DSS, LimeLightSubsystem LLS) {
        _bss = BSS;
        _dss = DSS;
        _lls = LLS;
        addRequirements(_dss);
        addRequirements(_bss);
    }

    @Override
    public void initialize() { // sets state
        tim.stop();
        tim.reset();
        iBallCount = _bss.ballCount;
        _bss.fireTheBigIron = true;
        loaded = false;
        //_bss.pidD.reset();
    } 

    @Override
    public void execute() { 
        // set flags
        Utils.Flags.targetDistance = _lls.metersToTarget();
        _bss.fireTheBigIron = true;

        // turn the robot towards the target
        if (_lls.targetFound) {
            _dss.arcadeDrive(0.0, pidt.calculate(_lls.tx));
            if (_lls.tx < 1.2) {
                pidt.reset();
                Flags.hoopLocked = true;
            } else Flags.hoopLocked = false;
            _bss.setAimDistance(_lls.metersToTarget());
        }

        // logic for when to finish
        if (!loaded && _bss.breachSensorFlag) {
            loaded = true;
        } else if (loaded && !_bss.breachSensorFlag) {
            loaded = false;
            ballShot++;
        }
    }

    @Override
    public void end(boolean interuppted) { // at end sets bc = 0 and cleans up
        _bss.fireTheBigIron = false;
        _bss.ballCount = 0;
    }

    @Override
    public boolean isFinished() {
        return ballShot == iBallCount; // finishes when all balls are shot
    }
}