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
    ArrayList<LEDStrip> ledStrips = new ArrayList<>();
    AddressableLEDBuffer filler = new AddressableLEDBuffer(0);

    /** Creates a new LEDUtility. */
    public LEDUtility(int _port) {
        addressableLED = new AddressableLED(_port);
    }

    public void addStrip(LEDStrip _strip) {
        ledStrips.add(_strip);
        overallLength = overallLength + _strip.getLength();
        setLength(overallLength);
    }

    public LEDStrip getStrip(int index) {
        return ledStrips.get(index);
    }

    public void setAll(LEDEffect _effect, Color _color) {
        ledStrips.forEach(strip -> {
            strip.setColor(_color);
            strip.setEffect(_effect);
        });
    }

    public void setAll(LEDEffect _effect) {
        ledStrips.forEach(strip -> {
            strip.setEffect(_effect);
        });
    }

    // DEFAULT LED PATTERN, CHANGE PER SEASON
    public void setDefault() {
        getStrip(0).setEffect(LEDEffect.PULSE);
        getStrip(0).setColor(LEDEffects.flytBlue);
        getStrip(1).setEffect(LEDEffect.PULSE);
        getStrip(1).setColor(LEDEffects.flytBlue);
    }

    private void setLength(int length) {
        filler = new AddressableLEDBuffer(length);
        addressableLED.setLength(length);
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
                LEDEffects.setFlashing(_strip, 10);
                break;
            case PULSE:
                LEDEffects.setPulsing(_strip, 50);
                break;
            case CHASING:
                LEDEffects.setChasing(_strip, 5);
                break;
            case ALLIANCE:
                LEDEffects.setAllianceColor(_strip);
                break;
            default:
                LEDEffects.setSolidColor(_strip);
                break;
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        try {
            ledStrips.forEach(strip -> {
                setStrip(strip);
                for (int i = strip.getStart(); i <= strip.getStop(); i++) {
                    filler.setLED(i, strip.getIndex(i - strip.getStart()));
                }
            });
            addressableLED.setData(filler);
            addressableLED.start();
        } catch (Exception e) {
            System.out.println("LED EXception: " + e);
        }
    }
}
