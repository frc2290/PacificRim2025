package frc.robot.io;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class DifferentialArmIOSimTest {
  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void positionChangesWithSetpoint() {
    DifferentialArmIOSim io = new DifferentialArmIOSim();
    double initial = io.getLeftPosition();
    io.setArmVelocitySetpoints(1000.0, 1000.0);
    for (int i = 0; i < 10; i++) {
      io.update();
    }
    assertTrue(io.getLeftPosition() > initial);
  }
}
