package frc.robot.io;

import java.util.List;

/** Minimal vision implementation for real hardware (no-op). */
public class VisionIOReal implements VisionIO {
  @Override
  public List<VisionMeasurement> getVisionMeasurements() {
    return List.of();
  }
}
