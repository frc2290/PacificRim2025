// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.util.Color;
import frc.utils.LEDEffects.LEDEffect;

/** Represents a named subset of the full LED buffer that can be animated independently. */
public class LEDStrip {
    /** Descriptive name so the coordinator can reference the strip. */
    String name;
    /** Starting index in the master LED buffer. */
    int start;
    /** Ending index (exclusive) in the master LED buffer. */
    int stop;
    /** View into the master buffer used to render effects. */
    AddressableLEDBufferView bufferView;
    /** Base color for effects when no override is supplied. */
    Color color = LEDEffects.flytBlue;
    /** Active animation effect to apply. */
    LEDEffect effect = LEDEffect.SOLID;
    /** Generic helper flag that allows custom behaviors to annotate strips. */
    boolean helperBool = false;

    public LEDStrip(String _name, AddressableLEDBuffer _buffer, int _start, int _stop) {
        name = _name;
        start = _start;
        stop = _stop;
        bufferView = _buffer.createView(start, stop);
    }
    
    public LEDStrip(String _name, AddressableLEDBuffer _buffer, int _start, int _stop, boolean _reversed) {
        name = _name;
        start = _start;
        stop = _stop;
        bufferView = _buffer.createView(start, stop);
        bufferView = bufferView.reversed();
    }

    public AddressableLEDBufferView getBufferView() {
        return bufferView;
    }

    public void setBufferView(AddressableLEDBuffer _buffer) {
        if (bufferView.isReversed()) {
            bufferView = _buffer.createView(start, stop);
            bufferView = bufferView.reversed();
        } else {
            bufferView = _buffer.createView(start, stop);
        }
    }

    public String getName() {
        return name;
    }

    public LEDEffect getEffect() {
        return effect;
    }

    public boolean getReversed() {
        return bufferView.isReversed();
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

    public int getLength() {
        return bufferView.getLength();
    }

    public void setEffect(LEDEffect _effect) {
        effect = _effect;
    }

    public void setEffect(LEDEffect _effect, Color _color) {
        effect = _effect;
        color = _color;
    }

    public void setColor(Color _color) {
        color = colorUtils.gammaCorrection(_color);
    }

    public boolean getHelperBool() {
        return helperBool;
    }

    public void setHelperBool(boolean helper) {
        helperBool = helper;
    }
}
