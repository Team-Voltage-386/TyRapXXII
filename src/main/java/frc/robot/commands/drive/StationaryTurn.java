package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.DriveConstants.*;

public class StationaryTurn extends CommandBase {

    private final DriveSubsystem _dss;
    private final PIDController pidt = new PIDController(tP,tI,tD);
    private Pose2d startPose = new Pose2d();
    private double angle;
    private final Boolean relTurn;

    public StationaryTurn(DriveSubsystem DSS,double value,Boolean relativeTurn) {
        relTurn = relativeTurn;
        angle = value;
        _dss = DSS;
        addRequirements(_dss);
    }

    @Override
    public void initialize() {
        pidt.reset();
        startPose = _dss.getPose();
        if (relTurn) {
            angle += startPose.getRotation().getDegrees();
            while (angle > 360) angle -= 360;
            while (angle < 0) angle += 360;
        }
    } 

    @Override
    public void execute() {
        double turn = MathUtil.clamp(pidt.calculate(_dss.getHeadingError(angle),0), -1*tC,tC);
        //if (angle > 35) pidt.setI(0);
        //else pidt.setI(tI);
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