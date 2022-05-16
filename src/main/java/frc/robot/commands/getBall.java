package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.Flags;
import frc.robot.subsystems.BigIronSubsystem;

/**Runs the intake and ball system until a new ball is collected
 * @author Carl C.
 * @deprecated redundant
 */
public class getBall extends CommandBase {

    private final BigIronSubsystem _bss;
    private int iBallCount = 0;
    private final Timer timer = new Timer();
    private final Timer timeOut = new Timer();
    private final double to;
    
    /**Creates a new getBall Instruction
     * @param BSS The BigIron
     * @param timeOut a timeout in case the ball isn't picked up
     * @deprecated redundant
    */
    public getBall(BigIronSubsystem BSS, double timeOut) {
        _bss = BSS;
        addRequirements(_bss);
        to = timeOut;
    }

    @Override
    public void initialize() {
        _bss.intakeUpdate(!_bss.intakeOut);
        iBallCount = _bss.ballCount;
        timer.stop();
        timer.reset();
        timeOut.reset();
        timeOut.start();
        Flags.complianceOverride = true;
    } 

    @Override
    public void execute() { // run the systems until the ball count increases
        _bss.intakeUpdate(false);
        if (_bss.ballCount > iBallCount || _bss.ballOnTheWay) timer.start();
    }

    @Override
    public void end(boolean interuppted) {
        timer.stop();
        timer.reset();
        Flags.complianceOverride = false;
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.2) || timeOut.hasElapsed(to); // when the timer has reached either 0.2 or the timeout stop
    }
}