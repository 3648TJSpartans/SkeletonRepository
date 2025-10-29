package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;

public class LedSubsystem extends SubsystemBase {

    public final AddressableLEDBuffer ledBuffer;
    // left buffers
    public final AddressableLEDBufferView buffer0;
    public final AddressableLEDBufferView buffer1;

    private final AddressableLED led;

    public LedSubsystem() {
        led = new AddressableLED(LedConstants.ledPWMID);
        ledBuffer = new AddressableLEDBuffer(LedConstants.ledLength);

        // Sub-buffers can be used to set different parts of the strip to different
        // colors or patterns. Since we are limited to a single LED strip along the
        // whole robot, these are often necessary.

        buffer0 = ledBuffer.createView(LedConstants.buffer1StartLeft, LedConstants.buffer1EndLeft);
        buffer1 = ledBuffer.createView(LedConstants.buffer2StartLeft, LedConstants.buffer2EndLeft);

        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

    }

    // Writes a specified LED pattern to a specified buffer. A bunch of patterns can be
    // found in LEDConstants, or you can make your own.
    public void setLedPattern(LEDPattern pattern, int buffer) {
        if (buffer == 0) {
            pattern.applyTo(buffer0);
        } else if (buffer == 1) {
            pattern.applyTo(buffer1);
        }
        led.setData(ledBuffer);
    }

    public void turnLedsOff() {

        LedConstants.noColor.applyTo(ledBuffer);
        led.setData(ledBuffer);

    }

    public void setGlobalPattern(LEDPattern pattern) {
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }
}
