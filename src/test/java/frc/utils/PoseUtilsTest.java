package frc.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class PoseUtilsTest {

  @AfterEach
  void resetAlliance() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
    DriverStationSim.notifyNewData();
  }

  @Test
  void getSpeakerTagRedAlliance() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();
    assertEquals(4, PoseUtils.getSpeakerTag());
  }

  @Test
  void getSpeakerTagBlueAlliance() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    assertEquals(7, PoseUtils.getSpeakerTag());
  }

  @Test
  void getSpeakerTagUnknownDefaultsToRed() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
    DriverStationSim.notifyNewData();
    assertEquals(4, PoseUtils.getSpeakerTag());
  }

  @Test
  void inRangeDetectsBounds() {
    assertTrue(PoseUtils.inRange(1));
    assertTrue(PoseUtils.inRange(6));
    assertTrue(PoseUtils.inRange(3.5));
  }

  @Test
  void inRangeRejectsOutOfBounds() {
    assertFalse(PoseUtils.inRange(0.5));
    assertFalse(PoseUtils.inRange(6.5));
  }
}
