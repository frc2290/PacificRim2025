package frc.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.wpilibj.util.Color;
import org.junit.jupiter.api.Test;

class ColorUtilsTest {

  @Test
  void gammaFunctionLowSegment() {
    double input = 0.03;
    double expected = input / 12.92;
    assertEquals(expected, colorUtils.gammaFunction(input), 1e-9);
  }

  @Test
  void gammaFunctionHighSegment() {
    double input = 0.5;
    double expected = Math.pow((input + 0.055) / 1.0555, 2.8);
    assertEquals(expected, colorUtils.gammaFunction(input), 1e-9);
  }

  @Test
  void gammaCorrectionAppliesToAllChannels() {
    Color original = new Color(0.5, 0.2, 0.9);
    Color corrected = colorUtils.gammaCorrection(original);
    assertEquals(colorUtils.gammaFunction(0.5), corrected.red, 1e-3);
    assertEquals(colorUtils.gammaFunction(0.2), corrected.green, 1e-3);
    assertEquals(colorUtils.gammaFunction(0.9), corrected.blue, 1e-3);
  }

  @Test
  void bgrConvertSwapsRedAndBlue() {
    Color original = new Color(0.1, 0.2, 0.3);
    Color converted = colorUtils.BGRconvert(original);
    assertEquals(original.blue, converted.red, 1e-9);
    assertEquals(original.green, converted.green, 1e-9);
    assertEquals(original.red, converted.blue, 1e-9);
  }
}
