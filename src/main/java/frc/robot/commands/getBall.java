package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BigIronSubsystem;

/**Runs the intake and ball system until a new ball is collected
 * @author Carl C.
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
    */
    public getBall(BigIronSubsystem BSS, double timeOut) {
        _bss = BSS;
        addRequirements(_bss);
        to = timeOut;
    }

    @Override
    public void initialize() {
        _bss.intakeDo(!_bss.intakeOut);
        _bss.runIntake(true);
        iBallCount = _bss.ballCount;
        timer.stop();
        timer.reset();
        timeOut.reset();
        timeOut.start();
    } 

    @Override
    public void execute() { // run the systems until the ball count increases
        _bss.intakeDo(false);
        if (_bss.ballCount > iBallCount || _bss.ballOnTheWay) timer.start();
    }

    @Override
    public void end(boolean interuppted) {
        _bss.runIntake(false);
        timer.stop();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.2) || timeOut.hasElapsed(to); // when the timer has reached either 0.2 or the timeout stop
    }
}