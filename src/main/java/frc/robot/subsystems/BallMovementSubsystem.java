package frc.robot.subsystems;

import frc.robot.Constants;
import static frc.robot.Constants.BallMovementConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class BallMovementSubsystem extends SubsystemBase {
    private final TalonSRX intakeMotor = new TalonSRX(kIntakeID);


    public BallMovementSubsystem() {
                
    }

    @Override
    public void periodic() {
        
    }

    public void runIntake(Boolean b) {
        if (b) intakeMotor.set(ControlMode.PercentOutput, kIntakePower);
        else intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void simulationPeriodic() {
            // This method will be called once per scheduler run during simulation
    }
}
