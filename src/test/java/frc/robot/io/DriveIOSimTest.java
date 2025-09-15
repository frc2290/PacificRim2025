package frc.robot.io;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Unit tests for the MapleSim DriveIO implementation. */
class DriveIOSimTest {
  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void batteryVoltageIsNonZero() {
    DriveIOSim io = new DriveIOSim();
    assertTrue(io.getBatteryVoltage() > 0.0);
  }

  @Test
  void providesModulePositions() {
    DriveIOSim io = new DriveIOSim();
    io.periodic();
    assertEquals(4, io.getModulePositions().length);
  }

  @Test
  void providesDriveAppliedOutputs() {
    DriveIOSim io = new DriveIOSim();
    io.periodic();
    assertEquals(4, io.getDriveAppliedOutputs().length);
  }
}
