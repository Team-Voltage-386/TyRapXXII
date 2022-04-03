package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** just waits
 * @author Carl C.
 */
public class Delay extends CommandBase {

    private final Timer timer = new Timer();
    private final double time;
    
    /**@param t seconds to wait
    */
    public Delay(double t) {
        time = t;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    } 

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interuppted) {
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }
}