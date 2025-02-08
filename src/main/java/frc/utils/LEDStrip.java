// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.utils.LEDEffects.LEDEffect;

/** Add your docs here. */
public class LEDStrip {
    int start;
    int stop;
    int helperPos = 0;
    boolean invert = false;
    AddressableLEDBuffer buffer;
    Color color = LEDEffects.rrGreen;
    LEDEffect effect = LEDEffect.SOLID;

    public LEDStrip(int _start, int _stop, boolean _invert) {
        start = _start;
        stop = _stop;
        invert = _invert;
        buffer = new AddressableLEDBuffer((stop - start) + 1);
    }

    public LEDStrip(int _start, int _stop) {
        start = _start;
        stop = _stop;
        buffer = new AddressableLEDBuffer((stop - start) + 1);
    }

    public LEDEffect getEffect() {
        return effect;
    }

    public boolean getInverted() {
        return invert;
    }

    public Color getColor() {
        return color;
    }

    public int getStart() {
        return start;
    }

    public int getStop() {
        return stop;
    }

    public int getHelperPos() {
        return helperPos;
    }

    public int getLength() {
        return buffer.getLength();
    }

    public void setEffect(LEDEffect _effect) {
        effect = _effect;
    }

    public void setInverted(boolean _invert) {
        invert = _invert;
    }

    public void setColor(Color _color) {
        color = colorUtils.gammaCorrection(_color);
    }

    public void setStart(int _start) {
        start = _start;
    }

    public void setStop(int _stop) {
        stop = _stop;
    }

    public void setHelperPos(int _helperPos) {
        helperPos = _helperPos;
    }

    public void setBuffer(AddressableLEDBuffer _buffer) {
        buffer = _buffer;
    }

    public Color getIndex(int index) {
        return buffer.getLED(index);
    }
}
