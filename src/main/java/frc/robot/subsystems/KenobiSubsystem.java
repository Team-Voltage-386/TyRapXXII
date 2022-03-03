// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//GAIN THE HIGH GROUND
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

  // pneumatics
  private final DoubleSolenoid arms = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM,
      kChannelClimbOut, kChannelClimbIn);

  // sensors
  private final DigitalInput pneumaticsLimitSensor = new DigitalInput(kPneumaticsDIOID);
  private final DigitalInput elevatorLowerLimitSensor = new DigitalInput(kElevatorLowerLimitDIOID);
  private final DigitalInput elevatorUpperLimitSensor = new DigitalInput(kElevatorUpperLimitDIOID);

  // booleans, since DIOs are inverted when you get them
  private boolean pneumaticsLimitBoolean, elevatorLowerLimitBoolean, elevatorUpperLimitBoolean;

  public void updateSensors() {
    pneumaticsLimitBoolean = !pneumaticsLimitSensor.get();
    elevatorLowerLimitBoolean = !elevatorLowerLimitSensor.get();
    elevatorUpperLimitBoolean = !elevatorUpperLimitSensor.get();
  }

  /** Creates a new Kenobi. */
  public KenobiSubsystem() {
    elevatorFollower.follow(elevatorLeader);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSensors();
    updateWidgets();
  }

  public void armsOut() {
    arms.set(Value.kForward);
    armsOut = true;
  }

  public void armsIn() {
    arms.set(Value.kReverse);
    armsOut = false;
  }

  protected boolean armsOut = false;

  public void armsDo() {
    if (armsOut) {
      armsIn();
    } else {
      armsOut();
    }
    armsOut = !armsOut;
  }

  public void elevatorDo(double power) {
    double output = power;
    boolean goingUp = power > 0.0;
    if ((elevatorUpperLimitBoolean && goingUp) || (elevatorLowerLimitBoolean && !goingUp))
      output = 0.0;
    elevatorLeader.set(output);
  }

  // shuffleboard
  private ShuffleboardTab tab = Shuffleboard.getTab("Climb");
  // widgets
  private NetworkTableEntry pneumaticsLimitWidget = tab.add("armsDown", false).withPosition(0, 0).withSize(1, 1)
      .getEntry();
  private NetworkTableEntry elevatorLowerLimitSensorWidget = tab.add("lower Limit", false).withPosition(1, 0)
      .withSize(1, 1).getEntry();
  private NetworkTableEntry elevatorUpperLimitSensorWidget = tab.add("upper Limit", false).withPosition(2, 0)
      .withSize(1, 1).getEntry();

  public void updateWidgets() {
    pneumaticsLimitWidget.setBoolean(pneumaticsLimitBoolean);
    elevatorLowerLimitSensorWidget.setBoolean(elevatorLowerLimitBoolean);
    elevatorUpperLimitSensorWidget.setBoolean(elevatorUpperLimitBoolean);
  }
}
