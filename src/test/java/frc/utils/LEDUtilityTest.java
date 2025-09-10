package frc.utils;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color;
import frc.utils.LEDEffects.LEDEffect;
import frc.utils.colorUtils;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class LEDUtilityTest {
  private AddressableLED led;
  private LEDUtility util;

  @BeforeEach
  void setUp() {
    led = mock(AddressableLED.class);
    util = new LEDUtility(led);
  }

  @Test
  void addStripUpdatesLengthAndRetrieval() {
    util.addStrip("Left", 0, 1);
    verify(led).setLength(2);
    LEDStrip left = util.getStrip("Left");
    assertNotNull(left);
    assertEquals(left, util.getStrip(0));
    assertNull(util.getStrip("Missing"));
  }

  @Test
  void setAllAppliesEffectAndColor() {
    util.addStrip("First", 0, 0);
    util.addStrip("Second", 1, 1);
    util.setAll(LEDEffect.SOLID, Color.kBlue);
    for (int i = 0; i < 2; i++) {
      LEDStrip strip = util.getStrip(i);
      assertEquals(LEDEffect.SOLID, strip.getEffect());
      assertEquals(colorUtils.gammaCorrection(Color.kBlue), strip.getColor());
    }
  }

  @Test
  void setDefaultAppliesPatterns() {
    util.addStrip("Left", 0, 0);
    util.addStrip("Right", 1, 1);
    util.addStrip("TopLeft", 2, 2);
    util.addStrip("TopRight", 3, 3);

    util.setDefault();

    assertEquals(LEDEffect.PULSE, util.getStrip("Left").getEffect());
    assertEquals(colorUtils.gammaCorrection(LEDEffects.flytBlue), util.getStrip("Left").getColor());
    assertEquals(LEDEffect.PULSE, util.getStrip("Right").getEffect());
    assertEquals(colorUtils.gammaCorrection(LEDEffects.flytBlue), util.getStrip("Right").getColor());
    assertEquals(LEDEffect.ALLIANCE, util.getStrip("TopLeft").getEffect());
    assertEquals(LEDEffect.ALLIANCE, util.getStrip("TopRight").getEffect());
  }

  @Test
  void setDefaultWithoutStripsDoesNotThrow() {
    assertDoesNotThrow(() -> util.setDefault());
  }

  @Test
  void periodicUpdatesLedHardware() {
    util.addStrip("Left", 0, 0);
    util.setAll(LEDEffect.SOLID, Color.kRed);
    util.periodic();
    verify(led, atLeastOnce()).setData(any());
    verify(led, atLeastOnce()).start();
  }
}

