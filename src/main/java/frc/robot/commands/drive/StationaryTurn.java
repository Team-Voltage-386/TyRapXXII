package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.DriveConstants.*;

/**
 * An instruction to rotate the robot
 * @author Carl C.
 */
public class StationaryTurn extends CommandBase {

    private final DriveSubsystem _dss;
    private double angle;
    private final Boolean relTurn;

    /**
     * Creates a new StationaryTurn instruction
     * @param DSS The drivesubsystem
     * @param value the number of degrees to rotate
     * @param relativeTurn if true, will rotate this number of clockwise degrees from current heading, else will rotate to heading relative to 
     * robot starting position
     */
    public StationaryTurn(DriveSubsystem DSS,double value,Boolean relativeTurn) {
        relTurn = relativeTurn;
        angle = value;
        _dss = DSS;
        addRequirements(_dss);
    }

    @Override
    public void initialize() { // configure starting state
        tPID.reset();
        if (relTurn) { // calculate heading hold
            angle += _dss.getPose().getRotation().getDegrees();
            while (angle > 360) angle -= 360;
            while (angle < 0) angle += 360;
        }
    } 

    @Override
    public void execute() {
        _dss.arcadeDrive(0.0, tsALG.get(_dss.getHeadingError(angle)));
    }

    @Override
    public void end(boolean interuppted) {
        _dss.arcadeDrive(0.0, 0.0); // stop the drives
    }

    @Override
    public boolean isFinished() {
        return Math.abs(_dss.getHeadingError(angle)) < 0.5 && _dss.getRotationSpeed() < 10; // finishes when the angle is within 0.5 and the robot is not rotating too quickly
    }
}