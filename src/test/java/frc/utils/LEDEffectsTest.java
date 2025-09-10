package frc.utils;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.util.Color;

class LEDEffectsTest {
    @Test
    void setSolidColorAppliesToBuffer() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(5);
        LEDStrip strip = new LEDStrip("test", buffer, 0, 4);
        LEDEffects.setSolidColor(strip, Color.kRed);
        assertEquals(Color.kRed, buffer.getLED(0));
    }

    // setAllianceColor is time-dependent; ensure no crash when alliance changes
    @Test
    void setAllianceColorDoesNotThrow() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(5);
        LEDStrip strip = new LEDStrip("test", buffer, 0, 4);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
        LEDEffects.setAllianceColor(strip);

        DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
        DriverStationSim.notifyNewData();
        LEDEffects.setAllianceColor(strip);
    }
}
