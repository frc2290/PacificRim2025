// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class LEDEffects {

    public static enum LEDEffect {
        SOLID,
        RAINBOW,
        RSL,
        FLASH,
        PULSE,
        CHASING,
        ALLIANCE
    }

    // Team Green
    public static Color rrGreen = new Color("#7CF205");
    public static Color flytBlue = new Color("#0081B3");

    public static void setSolidColor(LEDStrip _strip) {
        AddressableLEDBuffer output = new AddressableLEDBuffer(_strip.getLength());
        for (int i = 0; i < output.getLength(); i++) {
            output.setLED(i, _strip.getColor());
        }
        _strip.setBuffer(output);
    }

    public static void setSolidColor(LEDStrip _strip, Color _color) {
        AddressableLEDBuffer output = new AddressableLEDBuffer(_strip.getLength());
        for (int i = 0; i < output.getLength(); i++) {
            output.setLED(i, _color);
        }
        _strip.setBuffer(output);
    }

    public static void setHSVColor(LEDStrip _strip, int h, int s, int v) {
        AddressableLEDBuffer output = new AddressableLEDBuffer(_strip.getLength());
        for (int i = 0; i < output.getLength(); i++) {
            output.setHSV(i, h, s, v);
        }
        _strip.setBuffer(output);
    }

    public static void setRainbow(LEDStrip _strip) {
        AddressableLEDBuffer output = new AddressableLEDBuffer(_strip.getLength());
        // For every pixel
        if (_strip.getInverted()) {
            for (int i = _strip.getLength() - 1; i  >= 0; i--) {
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to precess
                final int hue = (_strip.getHelperPos() + (i * 180 / _strip.getLength()/2)) % 180;
                // Set the value
                output.setHSV(i, hue, 255, 128);
            }
        } else {
            for (int i = 0; i < _strip.getLength(); i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to precess
                final int hue = (_strip.getHelperPos() + (i * 180 / _strip.getLength()/2)) % 180;
                // Set the value
                output.setHSV(i, hue, 255, 128);
            }
        }
        // Increase by to make the rainbow "move"
        _strip.setHelperPos(_strip.getHelperPos() + 3);
        // Check bounds
        _strip.setHelperPos(_strip.getHelperPos() % 180);

        _strip.setBuffer(output);
    }

    public static void setRSLFlashing(LEDStrip _strip) {
        if (RobotController.getRSLState()) {
            setSolidColor(_strip, (DriverStation.getAlliance().get() == Alliance.Blue ? Color.kFirstBlue : Color.kFirstRed));
        } else {
            setSolidColor(_strip, rrGreen);
        }
    }

    public static void setFlashing(LEDStrip _strip, int _interval) {
        if (_strip.getHelperPos() == _interval) {
            Color temp = _strip.getIndex(0);
            if (temp.equals(_strip.getColor())) {
                setSolidColor(_strip, Color.kBlack);
            } else {
                setSolidColor(_strip, _strip.getColor());
            }
            _strip.setHelperPos(0);
        } else {
            _strip.setHelperPos(_strip.getHelperPos() + 1);
        }
    }

    // You only get red, blue, or green. Too bad, do it yourself then.
    public static void setPulsing(LEDStrip _strip, int _interval) {
        if (_strip.getHelperPos() == 255) {
            _strip.setHelperPos(128);
        } else if (_strip.getHelperPos() == 0) {
            _strip.setHelperPos(128);
        } else {
            _strip.setHelperPos(_strip.getHelperPos() + 1);
        }
        if (_strip.getColor().equals(Color.kFirstBlue)) {
            setHSVColor(_strip, 206, 100, _interval);
        } else if (_strip.getColor().equals(Color.kFirstRed)) {
            setHSVColor(_strip, 358, 88, _interval);
        } else {
            setHSVColor(_strip, 90, 98, _interval);
        }
    }

    public static void setChasing(LEDStrip _strip, int _interval) {
        AddressableLEDBuffer output = new AddressableLEDBuffer(_strip.getLength());
        int i = _strip.getHelperPos();
        output.setLED(i, Color.kBlack);
        if (_strip.getInverted()) {
            i--;
            if (i < 0) {
                i = _strip.getLength() - 1;
            }
        } else {
            i++;
            i %= _strip.getLength() - 1;
        }
        _strip.setHelperPos(i);
        output.setLED(i, _strip.getColor());
        _strip.setBuffer(output);
    }

    public static void setAllianceColor(LEDStrip _strip) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            setSolidColor(_strip, (alliance.get() == Alliance.Blue ? Color.kFirstBlue : Color.kFirstRed));
        } else {
            setSolidColor(_strip, Color.kFirstBlue);
        }
    }
}
