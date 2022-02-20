// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//GAIN THE HIGH GROUND
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KenobiConstants;

public class KenobiSubsystem extends SubsystemBase {
  // pneumatics
  DoubleSolenoid elevators = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM,
      KenobiConstants.kChannelClimbIn, KenobiConstants.kChannelClimbOUt);

  /** Creates a new Kenobi. */
  public KenobiSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void elevatorsOut() {
    elevators.set(Value.kForward);
  }

  public void elevatorsIn() {
    elevators.set(Value.kOff);
  }

  protected boolean elevatorOut = false;

  public void elevatorsDo() {
    if (elevatorOut) {
      elevatorsIn();
    } else {
      elevatorsOut();
    }
    elevatorOut = !elevatorOut;
  }
}
