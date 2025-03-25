// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;

/** Add your docs here. */
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
        Map<Double, Color> maskSteps = Map.of(0.0, (on ? Color.kBlack : Color.kWhite), 0.5, (on ? Color.kWhite : Color.kBlack));
        LEDPattern base = LEDPattern.solid(_strip.getColor());
        LEDPattern blink = base.blink(Seconds.of(_interval));
        LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(_interval * 2));
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
        LEDPattern mask = LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(_interval));
        base.mask(mask).applyTo(_strip.getBufferView());
    }

    public static void setAllianceColor(LEDStrip _strip) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            LEDPattern.solid((alliance.get() == Alliance.Blue ? Color.kFirstBlue : Color.kFirstRed)).breathe(Seconds.of(2)).applyTo(_strip.getBufferView());
        } else {
            LEDPattern.solid(Color.kWhite).applyTo(_strip.getBufferView());
        }
    }
}
