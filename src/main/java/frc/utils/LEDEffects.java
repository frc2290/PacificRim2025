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

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;

/** Central location for predefined LED animations used throughout the robot. */
public class LEDEffects {

  public static enum LEDEffect {
    SOLID,
    RAINBOW,
    RSL,
    FLASH,
    PULSE,
    CHASING,
    ALLIANCE,
    NAVLIGHTS
  }

  // Team Blue
  public static Color flytBlue = new Color("#0081B3");

  public static void setSolidColor(LEDStrip _strip) {
    LEDPattern.solid(_strip.getColor()).applyTo(_strip.getBufferView());
  }

  public static void setSolidColor(LEDStrip _strip, Color _color) {
    LEDPattern.solid(_color).applyTo(_strip.getBufferView());
  }

  public static void setHSVColor(LEDStrip _strip, int h, int s, int v) {
    LEDPattern.solid(Color.fromHSV(h, s, v)).applyTo(_strip.getBufferView());
  }

  public static void setRainbow(LEDStrip _strip) {
    LEDPattern.rainbow(255, 128).applyTo(_strip.getBufferView());
  }

  public static void setRSLFlashing(LEDStrip _strip) {
    LEDPattern base = LEDPattern.solid(flytBlue);
    base.synchronizedBlink(RobotController::getRSLState);
    base.applyTo(_strip.getBufferView());
  }

  public static void setFlashing(LEDStrip _strip, double _interval) {
    LEDPattern base = LEDPattern.solid(_strip.getColor());
    base.blink(Seconds.of(_interval)).applyTo(_strip.getBufferView());
  }

  public static void setNavLights(LEDStrip _strip, double _interval, boolean on) {
    Map<Double, Color> maskSteps =
        Map.of(0.0, (on ? Color.kBlack : Color.kWhite), 0.5, (on ? Color.kWhite : Color.kBlack));
    LEDPattern base = LEDPattern.solid(_strip.getColor());
    // LEDPattern blink = base.blink(Seconds.of(_interval));
    LEDPattern mask =
        LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(_interval * 2));
    base.mask(mask).applyTo(_strip.getBufferView());
  }

  // You only get red, blue, or green. Too bad, do it yourself then.
  public static void setPulsing(LEDStrip _strip, int _interval) {
    LEDPattern base = LEDPattern.solid(_strip.getColor());
    base.breathe(Seconds.of(_interval)).applyTo(_strip.getBufferView());
  }

  public static void setChasing(LEDStrip _strip, double _interval) {
    Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
    LEDPattern base = LEDPattern.solid(_strip.getColor());
    LEDPattern mask =
        LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(_interval));
    base.mask(mask).applyTo(_strip.getBufferView());
  }

  public static void setAllianceColor(LEDStrip _strip) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      LEDPattern.solid((alliance.get() == Alliance.Blue ? Color.kFirstBlue : Color.kFirstRed))
          .breathe(Seconds.of(2))
          .applyTo(_strip.getBufferView());
    } else {
      LEDPattern.solid(Color.kWhite).applyTo(_strip.getBufferView());
    }
  }
}
