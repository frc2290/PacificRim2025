package frc.robot.io;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;

/** Hardware-agnostic interface for vision systems providing global pose measurements. */
public interface VisionIO {
  /** Simple record describing a vision pose observation. */
  public static class VisionMeasurement {
    public final Pose2d pose;
    public final double timestamp;

    public VisionMeasurement(Pose2d pose, double timestamp) {
      this.pose = pose;
      this.timestamp = timestamp;
    }
  }

  /** Retrieve any new pose measurements since the last call. */
  default List<VisionMeasurement> getVisionMeasurements() {
    return List.of();
  }

  /** Called periodically for hardware-specific updates. */
  default void periodic() {}
}
