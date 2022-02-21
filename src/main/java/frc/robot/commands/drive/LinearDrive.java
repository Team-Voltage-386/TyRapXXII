package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.DriveConstants.*;

public class LinearDrive extends CommandBase {

    private final DriveSubsystem _dss;
    private final PIDController pidt = new PIDController(tP,tI,tD);
    private final PIDController pidd = new PIDController(dP, dI, dD);
    private Pose2d startPose = new Pose2d();
    private double headingHold = 0;
    private double distanceFromStart = 0;
    private final double targetDistance;
    private final Boolean angRel;
    
    /**@param angle angle set
     * @param rel if true, angle set is relative
    */
    public LinearDrive(DriveSubsystem DSS,double distance,double angle,Boolean rel) {
        angRel = rel;
        headingHold = angle;
        targetDistance = distance;
        _dss = DSS;
        addRequirements(_dss);
    }

    @Override
    public void initialize() {
        pidt.reset();
        pidd.reset();
        startPose = _dss.getPose();
        if (angRel) {
            headingHold += startPose.getRotation().getDegrees();
            while (headingHold > 360) headingHold -= 360;
            while (headingHold < 0) headingHold += 360;
        }
    } 

    @Override
    public void execute() {
        distanceFromStart = startPose.getTranslation().getDistance(_dss.getPose().getTranslation());
        _dss.arcadeDrive(-1*MathUtil.clamp(pidd.calculate(distanceFromStart,targetDistance),-1*dC,dC), MathUtil.clamp(pidt.calculate(_dss.getHeadingError(headingHold),0), -1*tC,tC));
    }

    @Override
    public void end(boolean interuppted) {
        _dss.arcadeDrive(0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(distanceFromStart-targetDistance) < 0.05;
    }
}