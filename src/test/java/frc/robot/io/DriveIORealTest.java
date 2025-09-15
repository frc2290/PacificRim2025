package frc.robot.io;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Basic tests for the real drivetrain IO. */
class DriveIORealTest {
  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void createsFourModulePositions() {
    DriveIOReal io = new DriveIOReal();
    SwerveModulePosition[] positions = io.getModulePositions();
    assertEquals(4, positions.length);
  }
}
