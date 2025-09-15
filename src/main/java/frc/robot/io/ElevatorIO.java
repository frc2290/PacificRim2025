package frc.robot.io;

/** Hardware-agnostic interface for elevator inputs and outputs. */
public interface ElevatorIO {
  /** Command the elevator with the specified voltage. */
  default void setVoltage(double volts) {}

  /** Get the elevator carriage position in meters. */
  default double getPositionMeters() {
    return 0.0;
  }

  /** Get the elevator carriage velocity in meters per second. */
  default double getVelocityMetersPerSec() {
    return 0.0;
  }

  /** Get the combined current draw of the elevator motors in amps. */
  default double getCurrentDrawAmps() {
    return 0.0;
  }

  /** Optional periodic hook for hardware-specific updates. */
  default void update() {}
}
