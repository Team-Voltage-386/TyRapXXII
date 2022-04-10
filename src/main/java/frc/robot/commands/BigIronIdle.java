package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BigIronSubsystem;

public class BigIronIdle extends CommandBase {

    private final BigIronSubsystem _bss;
    private boolean done = false;
    
    /**@param angle angle set
     * @param rel if true, angle set is relative
    */
    public BigIronIdle(BigIronSubsystem BSS) {
        _bss = BSS;
        addRequirements(_bss);
        done = false;
    }

    @Override
    public void initialize() {
        _bss.reset();
        _bss.intakeUpdate(_bss.intakeOut);
        _bss.fireTheBigIron = false;
        done = true;
    } 

    @Override
    public void execute() {
        _bss.intakeUpdate(_bss.intakeOut);
    }

    @Override
    public void end(boolean interuppted) {
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}