package frc.robot.commands.D;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BigIronSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.DriveConstants.*;

public class Delay extends CommandBase {

    private final BigIronSubsystem _bss;
    private boolean done = false;
    private final int target;
    
    /**@param angle angle set
     * @param rel if true, angle set is relative
    */
    public Delay(BigIronSubsystem BSS, int t) {
        _bss = BSS;
        target = t;
        addRequirements(_bss);
        done = false;
    }

    @Override
    public void initialize() {
        _bss.intakeDo(_bss.intakeOut);
        _bss.drumSP = target;
        _bss.fireTheBigIron = true;
        done = true;
    } 

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interuppted) {
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}