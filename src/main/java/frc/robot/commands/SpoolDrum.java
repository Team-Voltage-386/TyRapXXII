package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BigIronSubsystem;

public class SpoolDrum extends CommandBase {

    private final BigIronSubsystem _bss;
    private boolean done = false;
    private final int target;
    
    /**@param angle angle set
     * @param rel if true, angle set is relative
    */
    public SpoolDrum(BigIronSubsystem BSS, int t) {
        _bss = BSS;
        target = t;
        addRequirements(_bss);
        done = false;
    }

    @Override
    public void initialize() {
        _bss.intakeUpdate(_bss.intakeOut);
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