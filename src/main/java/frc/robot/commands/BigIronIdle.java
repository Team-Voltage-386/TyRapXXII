package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BigIronSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.DriveConstants.*;

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
        _bss.intakeDo(!_bss.intakeOut);
        _bss.fireTheBigIron = false;
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