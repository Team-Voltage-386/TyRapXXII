package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BigIronSubsystem;

public class getBall extends CommandBase {

    private final BigIronSubsystem _bss;
    private int iBallCount = 0;
    private final Timer timer = new Timer();
    
    /**@param angle angle set
     * @param rel if true, angle set is relative
    */
    public getBall(BigIronSubsystem BSS) {
        _bss = BSS;
        addRequirements(_bss);
    }

    @Override
    public void initialize() {
        //_bss.intakeDo(true);
        _bss.runIntake(true);
        iBallCount = _bss.ballCount;
        timer.stop();
        timer.reset();
    } 

    @Override
    public void execute() {
        _bss.intakeDo(false);
        if (_bss.ballCount > iBallCount) timer.start();
    }

    @Override
    public void end(boolean interuppted) {
        //_bss.intakeDo(true);
        _bss.runIntake(false);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.1);
    }
}