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
  private final DigitalInput pneumaticsLimit = new DigitalInput(kPneumaticsDIOID);
  private final DigitalInput elevatorSensor = new DigitalInput(kElevatorDIOID);

  /** Creates a new Kenobi. */
  public KenobiSubsystem() {
    elevatorFollower.follow(elevatorLeader);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateWidgets();
  }

  public void armsOut() {
    arms.set(Value.kForward);
  }

  public void armsIn() {
    arms.set(Value.kReverse);
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

  protected boolean hitLimitTop = true;// true = hit top limit, false = hit bottom limit
  protected boolean hitLimitAny = true;// limit sensor
  protected boolean movingUp = false;// based on power

  public void elevatorDo(double power) {
    double output = power;
    hitLimitAny = elevatorSensor.get();
    movingUp = (power > 0.0);
    if (hitLimitAny) {
      if (movingUp)
        hitLimitTop = true;
      else
        hitLimitTop = false;
    }
    if (hitLimitAny && ((hitLimitTop && movingUp) || (!hitLimitTop && !movingUp)))
      output = 0.0;
    elevatorLeader.set(output);
  }

  // shuffleboard
  private ShuffleboardTab tab = Shuffleboard.getTab("Climb");
  // widgets
  private NetworkTableEntry pneumaticsLimitWidget = tab.add("armsDown", false).withPosition(0, 0).withSize(1, 1)
      .getEntry();
  private NetworkTableEntry elevatorSensorWidget = tab.add("elevatorSensor", false).withPosition(1, 0).withSize(1, 1)
      .getEntry();

  public void updateWidgets() {
    pneumaticsLimitWidget.setBoolean(pneumaticsLimit.get());
    elevatorSensorWidget.setBoolean(elevatorSensor.get());
  }
}
