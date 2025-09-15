package frc.robot.io;

/** Hardware-agnostic interface for manipulator (gripper) IO. */
public interface ManipulatorIO {
  /** Set the intake motor voltage. */
  default void setVoltage(double volts) {}

  /** Motor position in rotations. */
  default double getPositionRotations() {
    return 0.0;
  }

  /** Absolute wrist position in rotations. */
  default double getWristPosition() {
    return 0.0;
  }

  /** Motor velocity in RPM. */
  default double getVelocityRPM() {
    return 0.0;
  }

  /** Current draw of the motor in amps. */
  default double getCurrentAmps() {
    return 0.0;
  }

  /** True if a coral game piece is currently held. */
  default boolean hasCoral() {
    return false;
  }

  /** Update the simulated coral possession state. */
  default void setCoral(boolean coral) {}

  /** True if an algae game piece is held. */
  default boolean hasAlgae() {
    return false;
  }

  default void setAlgae(boolean algae) {}

  /** Raw limit-switch style detection of coral presence. */
  default boolean seesCoral() {
    return false;
  }

  /** Reset the motor encoder position to zero. */
  default void resetPosition() {}

  /** Optional periodic update hook. */
  default void update() {}
}
