// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Controls the LED strip on the robot. Patterns are implemented here,
 * and they are switched by commands.
 * @author Hale Barber (H!)
 */
public class SubsystemLeds extends SubsystemBase {

  public enum Mode {
    Error,
    Blank,
    Red,
    HueCircle,
    TransFlag,
  }

  private void loadBufferHueCircle(int timer) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, (i * 2 + timer / 10) % 180, 255, 255);
    }
  }

  private void loadBufferTransFlag(int timer) {
    final Color8Bit blue = new Color8Bit(0, 200, 255);
    final Color8Bit pink = new Color8Bit(245, 65, 175);
    final Color8Bit white = new Color8Bit(200, 200, 200);
    final Color8Bit filler = new Color8Bit(0, 0, 0);

    final Color8Bit[] pattern = new Color8Bit[] {blue, blue, pink, pink, white, white, pink, pink, blue, blue, filler, filler, filler, filler};

    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, pattern[(i + timer / 50) % pattern.length]);
    }
  }

  private void loadBufferRed(int timer) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 255, 0, 0);
    }
  }

  private void loadBufferBlank(int timer) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 0);
    }
  }

  private void loadBufferError(int timer) {
    final Color8Bit yellow = new Color8Bit(255, 255, 0);
    final Color8Bit green = new Color8Bit(0, 120, 20);

    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, (i + timer / 50) % 2 == 0 ? yellow : green);
    }
  }

  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private Mode state = Mode.Error;
  private int timer = 0;
  /** Creates a new SubsystemLeds. */
  public SubsystemLeds() {
    led = new AddressableLED(0); // TODO move things to constants
    ledBuffer = new AddressableLEDBuffer(12);

    loadBufferBlank(0);

    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
  }

  @Override
  public void periodic() {

    if (timer % 10 == 0) {
      switch (state) {
        case Error:
          loadBufferError(timer);
          break;
        case Blank:
          loadBufferBlank(timer);
          break;
        case HueCircle:
          loadBufferHueCircle(timer);
          break;
        case Red:
          loadBufferRed(timer);
          break;
        case TransFlag:
          loadBufferTransFlag(timer);
          break;
      }

      led.setData(ledBuffer);
      led.start();
    }

    timer++;
  }
}
