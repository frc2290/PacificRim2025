package frc.robot.io;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class DifferentialArmIORealTest {
  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void constructsAndReadsPosition() {
    DifferentialArmIOReal io = new DifferentialArmIOReal();
    assertEquals(0.0, io.getLeftPosition(), 1e-9);
  }
}
