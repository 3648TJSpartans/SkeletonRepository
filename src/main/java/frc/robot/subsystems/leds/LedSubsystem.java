// Copyright (c) 2025-2026 First Robotics Competition Team 3648.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

  public final AddressableLEDBuffer ledBuffer;
  // left buffers
  public final AddressableLEDBufferView buffer1Left;
  public final AddressableLEDBufferView buffer2Left;
  public final AddressableLEDBufferView buffer3Left;
  public final AddressableLEDBufferView buffer4Left;
  // rightlbuffers
  public final AddressableLEDBufferView buffer1Right;
  public final AddressableLEDBufferView buffer2Right;
  public final AddressableLEDBufferView buffer3Right;
  public final AddressableLEDBufferView buffer4Right;

  private final AddressableLED led;

  public LedSubsystem() {
    led = new AddressableLED(LedConstants.ledPWMID);
    ledBuffer = new AddressableLEDBuffer(LedConstants.ledLength);

    // Create sub-buffers
    // left buffers
    buffer1Left = ledBuffer.createView(LedConstants.buffer1StartLeft, LedConstants.buffer1EndLeft);
    buffer2Left = ledBuffer.createView(LedConstants.buffer2StartLeft, LedConstants.buffer2EndLeft);
    buffer3Left = ledBuffer.createView(LedConstants.buffer3StartLeft, LedConstants.buffer3EndLeft);
    buffer4Left = ledBuffer.createView(LedConstants.buffer4StartLeft, LedConstants.buffer4EndLeft);
    // right buffers
    buffer1Right =
        ledBuffer.createView(LedConstants.buffer1StartRight, LedConstants.buffer1EndRight);
    buffer2Right =
        ledBuffer.createView(LedConstants.buffer2StartRight, LedConstants.buffer2EndRight);
    buffer3Right =
        ledBuffer.createView(LedConstants.buffer3StartRight, LedConstants.buffer3EndRight);
    buffer4Right =
        ledBuffer.createView(LedConstants.buffer4StartRight, LedConstants.buffer4EndRight);

    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  // Writes a specified LED pattern to the buffer. A bunch of patterns can be
  // found in LEDConstants, or you can make your own.
  public void setLedPattern(LEDPattern pattern, int level) {
    if (level == 1) {
      pattern.applyTo(buffer1Left);
      LedConstants.noColor.applyTo(buffer2Left);
      LedConstants.noColor.applyTo(buffer3Left);
      LedConstants.noColor.applyTo(buffer4Left);

      // right side
      pattern.applyTo(buffer4Right);
      LedConstants.noColor.applyTo(buffer2Right);
      LedConstants.noColor.applyTo(buffer3Right);
      LedConstants.noColor.applyTo(buffer1Right);
    }
    if (level == 2) {
      pattern.applyTo(buffer1Left);
      pattern.applyTo(buffer2Left);
      LedConstants.noColor.applyTo(buffer3Left);
      LedConstants.noColor.applyTo(buffer4Left);

      // right side
      pattern.applyTo(buffer3Right);
      pattern.applyTo(buffer4Right);
      LedConstants.noColor.applyTo(buffer1Right);
      LedConstants.noColor.applyTo(buffer2Right);
    }
    if (level == 3) {
      pattern.applyTo(buffer1Left);
      pattern.applyTo(buffer2Left);
      pattern.applyTo(buffer3Left);
      LedConstants.noColor.applyTo(buffer4Left);
      // right side
      pattern.applyTo(buffer4Right);
      pattern.applyTo(buffer2Right);
      pattern.applyTo(buffer3Right);
      LedConstants.noColor.applyTo(buffer1Right);
    }
    if (level == 4) {
      pattern.applyTo(buffer1Left);
      pattern.applyTo(buffer2Left);
      pattern.applyTo(buffer3Left);
      pattern.applyTo(buffer4Left);
      // right side
      pattern.applyTo(buffer1Right);
      pattern.applyTo(buffer2Right);
      pattern.applyTo(buffer3Right);
      pattern.applyTo(buffer4Right);
    }
    led.setData(ledBuffer);
  }

  public void turnLedsOff() {

    LedConstants.noColor.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  /**
   * Sets the entire LED strip to a given pattern, overwriting any previous data on the strip.
   *
   * @param pattern - The pattern to set the LEDs to.
   */
  public void setGlobalPattern(LEDPattern pattern) {
    pattern.applyTo(ledBuffer);
    led.setData(ledBuffer);
  }

  /**
   * Sets a single LED to a given RGB value.
   *
   * @param r - The red value (0-255).
   * @param g - The green value (0-255).
   * @param b - The blue value (0-255).
   * @param i - The index of the LED to set.
   */
  public void setSingleLed(int r, int g, int b, int i) {
    ledBuffer.setRGB(i, r, g, b);
    led.setData(ledBuffer);
  }
}
