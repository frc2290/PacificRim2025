package frc.robot.io;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ManipulatorIOSimTest {
  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void positionChangesWithVoltage() {
    ManipulatorIOSim io = new ManipulatorIOSim();
    io.setVoltage(3.0);
    double initial = io.getPositionRotations();
    io.update();
    assertNotEquals(initial, io.getPositionRotations());
  }
}
