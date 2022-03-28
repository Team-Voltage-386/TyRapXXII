// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//GAIN THE HIGH GROUND
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.KenobiConstants.*;

public class KenobiSubsystem extends SubsystemBase {
  // motors
  private final CANSparkMax elevatorLeader = new CANSparkMax(kElevatorLeaderID, MotorType.kBrushless);
  private final CANSparkMax elevatorFollower = new CANSparkMax(kElevatorFollowerID, MotorType.kBrushless);

  // sensors
  private final DigitalInput pneumaticsLimitSensor = new DigitalInput(kPneumaticsDIOID);
  private final DigitalInput elevatorLowerLimitSensor = new DigitalInput(kElevatorLowerLimitDIOID);
  private final DigitalInput elevatorUpperLimitSensor = new DigitalInput(kElevatorUpperLimitDIOID);

  // booleans, since DIOs are inverted when you get them
  private boolean pneumaticsLimitBoolean = false;
  private boolean calibrated = false;
  public boolean elevatorLowLimitFlag = false;
  public boolean elevatorUpperLimitFlag = false;

  private void updateSensors() {
    pneumaticsLimitBoolean = !pneumaticsLimitSensor.get();
    elevatorLowLimitFlag = !elevatorLowerLimitSensor.get();
    elevatorUpperLimitFlag = !elevatorUpperLimitSensor.get();
  }

  /** Creates a new Kenobi. */
  public KenobiSubsystem() {
    elevatorFollower.follow(elevatorLeader);
    calibrated = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSensors();
    if (!calibrated) {
      setElePower(-0.2);
      if (elevatorLowLimitFlag) {
        calibrated = true;
        setElePower(0);
        getEnc().setPosition(0);
      }
    }

  }

/**@param power negative is up, positive is down */
  public void setElePower(double power) {
    if (calibrated) {
      if (elevatorLowLimitFlag) elevatorLeader.set(MathUtil.clamp(power, 0.0, 1.0)); // they work, don't mess with it lol
      else if(elevatorUpperLimitFlag) elevatorLeader.set(MathUtil.clamp(power, -1.0, 0.0));
      else elevatorLeader.set(power);
    } else elevatorLeader.set(-0.2);
  }

  public RelativeEncoder getEnc() {
    return elevatorLeader.getEncoder();
  }
}
