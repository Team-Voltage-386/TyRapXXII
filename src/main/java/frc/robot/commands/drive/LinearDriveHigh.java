package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.DriveConstants.*;

/** Drives the robot a set distance (note, distances here will NOT correspond to limelight distances, they are calibrated differently and neither is perfect meters). 
 * Also of note, this instruction only drives the robot until the odometry distance from start is correct, it holds the heading of the robot,
 *  but does not drive to a specified point. If the robot is bumped, or experiences major drive error, even if only momentarily, the final position 
 * can be affected drastically.
 * @author Carl C.
*/
public class LinearDriveHigh extends CommandBase {

    private final DriveSubsystem _dss;
    private Pose2d startPose = new Pose2d();
    private double headingHold = 0;
    private double distanceFromStart = 0;
    private double drive = 0;
    private final double targetDistance;
    private final Boolean angRel;
    private final Timer finTimer = new Timer();
    private final double d;
    
    /**Creates a new LinearDrive instrucction
     * @param DSS the drivesubsystem
     * @param distance the distance to drive (if calibrated, roughly in meters)
     * @param angle angle set
     * @param rel if true, angle set is relative
     * @param dir a 1 or -1 for forwards or backwards (could be enum but I'm lazy)
    */
    public LinearDriveHigh(DriveSubsystem DSS,double distance,double angle,Boolean rel,double dir) {
        angRel = rel;
        headingHold = angle;
        targetDistance = distance;
        _dss = DSS;
        d = dir;
        addRequirements(_dss);
        finTimer.stop();
        finTimer.reset();
    }

    @Override
    public void initialize() {
        _dss.setHighGear(true); // checks low gear and resets systems
        tPID.reset();
        startPose = _dss.getPose();
        drive = 0;
        finTimer.stop();
        finTimer.reset();
        if (angRel) { // calculate angle setpoint
            headingHold += startPose.getRotation().getDegrees();
            while (headingHold > 360) headingHold -= 360;
            while (headingHold < 0) headingHold += 360;
        }
    } 

    @Override
    public void execute() {
        distanceFromStart = startPose.getTranslation().getDistance(_dss.getPose().getTranslation());
        if (distanceFromStart > targetDistance - 0.25) { // drive until the distance from the start is within 0.25 of target, then start timer
            finTimer.start();
            drive = Utils.lerpA(drive, 0, kAutoDriveSmoothing);
        } else {
            drive = Utils.lerpA(drive, -1*getDrivePower(targetDistance-distanceFromStart), kAutoDriveSmoothing);
            finTimer.reset();
        }
        _dss.arcadeDrive(drive*d, tALG.get(_dss.getHeadingError(headingHold)));
    }

    @Override
    public void end(boolean interuppted) { // at end stop drive
        _dss.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() { // timer ensures the robot stops properly (smoothing is applied)
        return finTimer.hasElapsed(0.8);
    }
    /**
     * gets the drive error from the array of distances and powers, kinda like the shooter calibration
     * @param distErr the difference between the distance from start and the instructed drive distance
     * @return the strength with which to power the drive motors. -1 <-> 1
     */
    private double getDrivePower(double distErr) { 
        int ind = kDriveDistances.length-1;
        for (int i = 1; i < kDriveDistances.length; i++) if (distErr < kDriveDistances[i]) ind = i;
        double upper = kDriveDistances[ind];
        double lower = kDriveDistances[ind-1];
        double lf = (distErr-lower)/(upper-lower);
        return Utils.lerpA(kDrivePowers[ind-1], kDrivePowers[ind], lf);
    }
}