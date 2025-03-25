// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LEDEffects.LEDEffect;

public class LEDUtility extends SubsystemBase {
    AddressableLED addressableLED;
    int overallLength = 0;
    AddressableLEDBuffer filler = new AddressableLEDBuffer(0);
    ArrayList<LEDStrip> newLedStrips = new ArrayList<>();

    /** Creates a new LEDUtility. */
    public LEDUtility(int _port) {
        addressableLED = new AddressableLED(_port);
        addressableLED.start();
    }

    public void addStrip(String _name, int start, int end) {
        overallLength = overallLength + ((end - start) + 1);
        setLength(overallLength);
        newLedStrips.add(new LEDStrip(_name, filler, start, end));
    }

    public LEDStrip getStrip(int index) {
        return newLedStrips.get(index);
    }

    public LEDStrip getStrip(String name) {
        for (LEDStrip strip : newLedStrips) {
            if (strip.getName() == name) {
                return strip;
            }
        }
        return null;
    }

    public void setAll(LEDEffect _effect, Color _color) {
        newLedStrips.forEach(strip -> {
            strip.setColor(_color);
            strip.setEffect(_effect);
        });
    }

    public void setAll(LEDEffect _effect) {
        newLedStrips.forEach(strip -> {
            strip.setEffect(_effect);
        });
    }

    // DEFAULT LED PATTERN, CHANGE PER SEASON
    public void setDefault() {
        getStrip("Left").setEffect(LEDEffect.PULSE);
        getStrip("Left").setColor(LEDEffects.flytBlue);
        getStrip("Right").setEffect(LEDEffect.PULSE);
        getStrip("Right").setColor(LEDEffects.flytBlue);
        getStrip("TopLeft").setEffect(LEDEffect.ALLIANCE);
        getStrip("TopRight").setEffect(LEDEffect.ALLIANCE);
    }

    private void setLength(int length) {
        filler = new AddressableLEDBuffer(length);
        addressableLED.setLength(length);
        for (LEDStrip strip : newLedStrips) {
            strip.setBufferView(filler);
        }
    }

    private Color getAlliance() {
        return DriverStation.getAlliance().get() == Alliance.Blue ? Color.kFirstBlue : Color.kFirstRed;
    }

    private void setStrip(LEDStrip _strip) {
        switch (_strip.getEffect()) {
            case SOLID:
                LEDEffects.setSolidColor(_strip);
                break;
            case RAINBOW:
                LEDEffects.setRainbow(_strip);
                break;
            case RSL:
                LEDEffects.setRSLFlashing(_strip);
                break;
            case FLASH:
                LEDEffects.setFlashing(_strip, 0.05);
                break;
            case PULSE:
                LEDEffects.setPulsing(_strip, 2);
                break;
            case CHASING:
                LEDEffects.setChasing(_strip, 0.25);
                break;
            case ALLIANCE:
                LEDEffects.setAllianceColor(_strip);
                break;
            case NAVLIGHTS:
                LEDEffects.setNavLights(_strip, 1, _strip.getHelperBool());
            default:
                LEDEffects.setSolidColor(_strip);
                break;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        try {
            newLedStrips.forEach(strip -> {
                setStrip(strip);
            });
            addressableLED.setData(filler);
            addressableLED.start();
        } catch (Exception e) {
            System.out.println("LED EXception: " + e);
        }
    }
}
