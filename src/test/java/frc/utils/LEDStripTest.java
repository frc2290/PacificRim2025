package frc.utils;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

class LEDStripTest {
    @Test
    void setColorAppliesGammaCorrection() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(5);
        LEDStrip strip = new LEDStrip("test", buffer, 0, 4);
        Color raw = new Color(0.2, 0.2, 0.2);
        strip.setColor(raw);
        assertEquals(colorUtils.gammaCorrection(raw), strip.getColor());
    }

    @Test
    void reversedConstructorSetsReversedView() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(5);
        LEDStrip strip = new LEDStrip("rev", buffer, 0, 4, true);
        assertTrue(strip.getReversed());
        assertEquals(5, strip.getLength());
    }

    @Test
    void setBufferViewPreservesReversal() {
        AddressableLEDBuffer buffer1 = new AddressableLEDBuffer(5);
        LEDStrip strip = new LEDStrip("rev", buffer1, 0, 4, true);
        AddressableLEDBuffer buffer2 = new AddressableLEDBuffer(5);
        strip.setBufferView(buffer2);
        assertTrue(strip.getReversed());
    }
}
