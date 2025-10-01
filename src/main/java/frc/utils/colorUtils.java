// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
package frc.utils;

import edu.wpi.first.wpilibj.util.Color;

/** Helper methods for converting and gamma-correcting LED colors. */
public class colorUtils {
  /** Im lazy and dont want to put this in for R,G,and B. So I made a new method. */
  public static double gammaFunction(double colorComponent) {
    if (colorComponent <= 0.04045) {
      return colorComponent / 12.92;
    } else {
      return Math.pow((colorComponent + 0.055) / 1.0555, 2.8);
    }
  }

  /**
   * sRGB is the normal RGB we are used to seeing. From 0 to 255. sRGB assumes a nonlinear curve
   * between brightness and R,G,B value This function will take a sRGB color like we are used to
   * seeing and turn it into what the LED strips should show to be the right color
   */
  public static Color gammaCorrection(Color sRGBcolor) {
    double newRed = gammaFunction(sRGBcolor.red);
    double newGreen = gammaFunction(sRGBcolor.green);
    double newBlue = gammaFunction(sRGBcolor.blue);
    return new Color(newRed, newGreen, newBlue);
  }

  /**
   * Color flippy flopy swipy swapy. Takes in RGB values and turns it into BGR values. So our
   * underglow matches. I think it was BGR but I could be wrong.
   */
  public static Color BGRconvert(Color RGB) {
    return new Color(RGB.blue, RGB.green, RGB.red);
  }
}
