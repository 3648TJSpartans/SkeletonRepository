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

import static org.wpilib.units.Units.*;

import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.util.Color;

public final class LedConstants {

  // Values //
  public static int ledLength = 120;
  public static final int ledPWMID = 1;
  public static final int buffer1StartLeft = 0;
  public static final int buffer1EndLeft = 14;
  public static final int buffer2StartLeft = 15;
  public static final int buffer2EndLeft = 29;
  public static final int buffer3StartLeft = 30;
  public static final int buffer3EndLeft = 44;
  public static final int buffer4StartLeft = 45;
  public static final int buffer4EndLeft = 59;

  public static int statusCheckOffset = 0;
  // right side
  public static final int buffer1StartRight = 60;
  public static final int buffer1EndRight = 74;
  public static final int buffer2StartRight = 75;
  public static final int buffer2EndRight = 89;
  public static final int buffer3StartRight = 90;
  public static final int buffer3EndRight = 104;
  public static final int buffer4StartRight = 105;
  public static final int buffer4EndRight = 119;
  // Colors & Patterns //

  // Solid Colors
  public static LEDPattern green = LEDPattern.solid(Color.GREEN);
  public static LEDPattern red = LEDPattern.solid(Color.RED);
  public static LEDPattern blue = LEDPattern.solid(Color.BLUE);
  public static LEDPattern teal = LEDPattern.solid(Color.TEAL);
  public static LEDPattern yellow = LEDPattern.solid(Color.YELLOW);
  public static LEDPattern purple = LEDPattern.solid(Color.PURPLE);
  public static LEDPattern white = LEDPattern.solid(Color.WHITE);
  public static LEDPattern noColor = LEDPattern.solid(Color.BLACK);
  // Gradients
  public static LEDPattern rainbow = LEDPattern.rainbow(255, 128);
  public static LEDPattern purpleGradient =
      LEDPattern.gradient(LEDPattern.GradientType.DISCONTINUOUS, Color.RED, Color.BLUE);
  public static LEDPattern elevatorGradient =
      LEDPattern.gradient(
          LEDPattern.GradientType.DISCONTINUOUS,
          Color.RED,
          Color.YELLOW,
          Color.GREEN,
          Color.TEAL); // Stoplight-esque gradient to use on the elevator

  // Animated Patterns
  public static LEDPattern breathingGreen = green.breathe(Seconds.of(2));
  public static LEDPattern blinkingBlue = blue.breathe(Seconds.of(1.5));
  public static LEDPattern blinkingteal = teal.breathe(Seconds.of(1.5));
}
