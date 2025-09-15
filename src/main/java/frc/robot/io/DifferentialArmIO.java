package frc.robot.io;

/** Hardware-agnostic interface for the differential arm. */
public interface DifferentialArmIO extends AutoCloseable {
  /** Command left and right sides with velocity setpoints. */
  default void setArmVelocitySetpoints(double left, double right) {}

  /** Get left motor position in native units. */
  default double getLeftPosition() {
    return 0.0;
  }

  /** Get right motor position in native units. */
  default double getRightPosition() {
    return 0.0;
  }

  /** Get left motor velocity in native units per second. */
  default double getLeftVelocity() {
    return 0.0;
  }

  /** Get right motor velocity in native units per second. */
  default double getRightVelocity() {
    return 0.0;
  }

  /** Get current draw of the left motor in amps. */
  default double getLeftCurrentAmps() {
    return 0.0;
  }

  /** Get current draw of the right motor in amps. */
  default double getRightCurrentAmps() {
    return 0.0;
  }

  /** Latest LaserCAN distance measurement in millimeters. */
  default int getLaserDistanceMm() {
    return 0;
  }

  /** Whether a valid LaserCAN measurement is available. */
  default boolean hasLaserDistance() {
    return false;
  }

  /** Optional periodic hook for hardware-specific updates. */
  default void update() {}

  @Override
  default void close() {}
}
