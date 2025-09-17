package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.DifferentialArm;

/** Unit tests covering the command generation inside {@link DifferentialSubsystem}. */
public class DifferentialSubsystemTest {
  @Test
  public void positiveRotationSetpointCommandsFasterRightSide() {
    try (DifferentialSubsystem diff = new DifferentialSubsystem()) {
      diff.setExtensionSetpoint(0.0);
      diff.setRotationSetpoint(30.0);
      diff.periodic();
      assertTrue(
          diff.getRightVelocityCommand() > diff.getLeftVelocityCommand(),
          "Positive rotation should speed up the right side relative to the left");
    }
  }

  @Test
  public void negativeRotationSetpointCommandsFasterLeftSide() {
    try (DifferentialSubsystem diff = new DifferentialSubsystem()) {
      diff.setExtensionSetpoint(0.0);
      diff.setRotationSetpoint(-30.0);
      diff.periodic();
      assertTrue(
          diff.getLeftVelocityCommand() > diff.getRightVelocityCommand(),
          "Negative rotation should speed up the left side relative to the right");
    }
  }

  @Test
  public void rotationCommandClampedToMotorFreeSpeed() {
    try (DifferentialSubsystem diff = new DifferentialSubsystem()) {
      diff.setExtensionSetpoint(0.0);
      diff.setRotationSetpoint(10_000.0);
      double peakRight = 0.0;
      double peakLeft = 0.0;
      for (int i = 0; i < 200; ++i) {
        diff.periodic();
        peakRight = Math.max(peakRight, Math.abs(diff.getRightVelocityCommand()));
        peakLeft = Math.max(peakLeft, Math.abs(diff.getLeftVelocityCommand()));
      }
      double maxSpoolVelocity =
          DifferentialArm.kSimMotor.freeSpeedRadPerSec
              * DifferentialArm.kSimLinearDriveRadiusMeters
              * 1000.0;
      assertEquals(
          maxSpoolVelocity,
          peakRight,
          1e-6,
          "Rotation command should not exceed motor free speed on the right spool");
      assertEquals(
          maxSpoolVelocity,
          peakLeft,
          1e-6,
          "Rotation command should not exceed motor free speed on the left spool");
      assertTrue(
          diff.getRightVelocityCommand() > 0.0
              && diff.getLeftVelocityCommand() < 0.0,
          "Positive rotation setpoint should command opposite spool directions");
    }
  }
}
