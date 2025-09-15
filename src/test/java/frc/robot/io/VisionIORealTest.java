package frc.robot.io;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/** Basic tests for the real vision IO implementation. */
class VisionIORealTest {
  @Test
  void returnsEmptyMeasurementList() {
    VisionIOReal vision = new VisionIOReal();
    assertTrue(vision.getVisionMeasurements().isEmpty());
  }
}
