// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import static frc.robot.Constants.LEDConstants.*;

public class LEDSubsystem extends SubsystemBase {
  AddressableLED ledRight = new AddressableLED(kRightStrip);
  AddressableLED ledLeft = new AddressableLED(kLeftStrip);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(kLEDLength);

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    ledRight.setLength(kLEDLength);
    ledLeft.setLength(kLEDLength);
    ledRight.setData(ledBuffer);
    ledLeft.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startLED() {
    ledLeft.start();
    ledRight.start();
  }

  public void stopLED() {
    ledLeft.stop();
    ledRight.stop();
  }

  public void setLEDSegment(int indexStart, int indexStop, Color color) {
    for (int i = indexStart; i <= indexStop; i++) {
      ledBuffer.setLED(i, color);
    }

  }

  public void setLEDSegmentOff(int indexStart, int indexStop) {
    for (int i = indexStart; i <= indexStop; i++) {
      ledBuffer.setRGB(i, 0, 0, 0);
    }
  }

}
