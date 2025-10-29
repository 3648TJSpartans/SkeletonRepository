package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import static edu.wpi.first.units.Units.*;

public final class LedConstants {

        public static int ledLength = 3;
        public static final int ledPWMID = 1;
        public static final int buffer0Start = 0;
        public static final int buffer0End = 1;
        public static final int buffer1Start = 2;
        public static final int buffer1End = 3;

        public static LEDPattern green = LEDPattern.solid(Color.kGreen);
        public static LEDPattern red = LEDPattern.solid(Color.kRed);
        public static LEDPattern blue = LEDPattern.solid(Color.kBlue);
        public static LEDPattern teal = LEDPattern.solid(Color.kTeal);
        public static LEDPattern yellow = LEDPattern.solid(Color.kYellow);
        public static LEDPattern purple = LEDPattern.solid(Color.kPurple);
        public static LEDPattern white = LEDPattern.solid(Color.kWhite);
        public static LEDPattern noColor = LEDPattern.solid(Color.kBlack);

        public static LEDPattern rainbow = LEDPattern.rainbow(255, 128);
        public static LEDPattern breathingGreen = green.breathe(Seconds.of(2));
        public static LEDPattern blinkingBlue = blue.breathe(Seconds.of(1.5));
}
