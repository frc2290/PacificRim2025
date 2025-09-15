package frc.robot.io;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ElevatorIOSimTest {
  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void positionMovesWithVoltage() {
    ElevatorIOSim io = new ElevatorIOSim();
    double initial = io.getPositionMeters();
    io.setVoltage(2.0);
    io.update();
    assertNotEquals(initial, io.getPositionMeters());
  }
}
