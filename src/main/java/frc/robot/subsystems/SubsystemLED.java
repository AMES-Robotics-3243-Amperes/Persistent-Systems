// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemLED extends SubsystemBase {
  protected AddressableLED led;
  protected AddressableLEDBuffer buffer;
  protected int counter = 0;
  protected final int length = 56;
  protected final Color[] transFlag = new Color[] {
    new Color(0, 108, 127),
    new Color(127, 0, 123),
    new Color(255, 255, 255),
    new Color(127, 0, 123),
    new Color(0, 108, 127),
    new Color(0, 0, 0),
    new Color(0, 0, 0)
  };
  /** Creates a new SubsystemLED. */
  public SubsystemLED() {
    led = new AddressableLED(0);
    led.setLength(length);
    buffer = new AddressableLEDBuffer(length);
  }

  @Override
  public void periodic() {
    /*for (int i = 0; i < length; i++) {
      buffer.setLED(i, Color.fromHSV((i*10 + counter) % 180, 230, 200));
    }
    
    counter%=180;*///Rainbow
    
    counter++;
    //buffer.setLED(0, new Color(0, 255, 0));
    if (counter % 5 == 0) {
      for (int i = 0; i < length; i++) {
        buffer.setLED(i, transFlag[((i/2) + (counter/50)) % transFlag.length]);
      }
      led.setData(buffer);
      led.start();
    }
    
  }
}
