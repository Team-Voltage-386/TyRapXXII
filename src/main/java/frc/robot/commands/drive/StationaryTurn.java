package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.DriveConstants.*;

/**
 * An instruction to rotate the robot
 * @author Carl C.
 */
public class StationaryTurn extends CommandBase {

    private final DriveSubsystem _dss;
    private final PIDController pidt = new PIDController(tP,tI,tD);
    private Pose2d startPose = new Pose2d();
    private double angle;
    private final Boolean relTurn;
    private double dir = 0;

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
    public void initialize() { // get start position and configure starting state
        pidt.reset();
        startPose = _dss.getPose();
        if (relTurn) { // calculate heading hold
            angle += startPose.getRotation().getDegrees();
            while (angle > 360) angle -= 360;
            while (angle < 0) angle += 360;
        }
    } 

    @Override
    public void execute() {
        double v = _dss.getHeadingError(angle);
        dir = v/Math.abs(v);
        double turn = MathUtil.clamp(pidt.calculate(v,0), -1*tC,tC) - (dir * 0.4);
        _dss.arcadeDrive(0.0, turn);
    }

    @Override
    public void end(boolean interuppted) {
        _dss.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(_dss.getHeadingError(angle)) < 0.5 && _dss.getRotationSpeed() < 10;
    }
}