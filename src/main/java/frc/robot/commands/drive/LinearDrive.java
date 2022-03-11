package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.DriveConstants.*;


public class LinearDrive extends CommandBase {

    private final DriveSubsystem _dss;
    private final PIDController pidt = new PIDController(tP,tI,tD);
    private Pose2d startPose = new Pose2d();
    private double headingHold = 0;
    private double distanceFromStart = 0;
    private double drive = 0;
    private final double targetDistance;
    private final Boolean angRel;
    private final Timer finTimer = new Timer();
    private final double d;
    
    /**@param angle angle set
     * @param rel if true, angle set is relative
    */
    public LinearDrive(DriveSubsystem DSS,double distance,double angle,Boolean rel,double dir) {
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
        _dss.setHighGear(false);
        pidt.reset();
        startPose = _dss.getPose();
        drive = 0;
        finTimer.stop();
        finTimer.reset();
        if (angRel) {
            headingHold += startPose.getRotation().getDegrees();
            while (headingHold > 360) headingHold -= 360;
            while (headingHold < 0) headingHold += 360;
        }
    } 

    @Override
    public void execute() {
        distanceFromStart = startPose.getTranslation().getDistance(_dss.getPose().getTranslation());
        if (distanceFromStart > targetDistance - 0.25) {
            finTimer.start();
            drive = Utils.lerpA(drive, 0, kAutoDriveSmoothing);
        } else {
            drive = Utils.lerpA(drive, -1*getDrivePower(targetDistance-distanceFromStart), kAutoDriveSmoothing);
            finTimer.reset();
        }
        _dss.arcadeDrive(drive*d, MathUtil.clamp(pidt.calculate(_dss.getHeadingError(headingHold),0), -1*tC,tC));
    }

    @Override
    public void end(boolean interuppted) {
        _dss.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return finTimer.hasElapsed(0.4);
    }

    private double getDrivePower(double distErr) {
        int ind = kDriveDistances.length-1;
        for (int i = 1; i < kDriveDistances.length; i++) if (distErr < kDriveDistances[i]) ind = i;
        double upper = kDriveDistances[ind];
        double lower = kDriveDistances[ind-1];
        double lf = (distErr-lower)/(upper-lower);
        return Utils.lerpA(kDrivePowers[ind-1], kDrivePowers[ind], lf);
    }
}