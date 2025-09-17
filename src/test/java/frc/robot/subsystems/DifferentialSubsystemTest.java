package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Constants.DifferentialArm;
import org.junit.jupiter.api.Test;

/** Unit tests covering the command generation inside {@link DifferentialSubsystem}. */
public class DifferentialSubsystemTest {
  @Test
  public void positiveRotationSetpointCommandsFasterLeftSide() {
    try (DifferentialSubsystem diff = new DifferentialSubsystem()) {
      diff.setExtensionSetpoint(0.0);
      diff.setRotationSetpoint(30.0);
      diff.periodic();
      assertTrue(
          diff.getLeftVelocityCommand() > diff.getRightVelocityCommand(),
          "Positive rotation should speed up the left side relative to the right");
    }
  }

  @Test
  public void negativeRotationSetpointCommandsFasterRightSide() {
    try (DifferentialSubsystem diff = new DifferentialSubsystem()) {
      diff.setExtensionSetpoint(0.0);
      diff.setRotationSetpoint(-30.0);
      diff.periodic();
      assertTrue(
          diff.getRightVelocityCommand() > diff.getLeftVelocityCommand(),
          "Negative rotation should speed up the right side relative to the left");
    }
  }

  @Test
  public void rotationCommandClampedToMotorFreeSpeed() throws ReflectiveOperationException {
    try (DifferentialSubsystem diff = new DifferentialSubsystem()) {
      diff.setExtensionSetpoint(0.0);
      diff.setRotationSetpoint(10_000.0);
      double peakRotationCommand = 0.0;
      var rotationField = DifferentialSubsystem.class.getDeclaredField("rotationVelocity");
      rotationField.setAccessible(true);
      for (int i = 0; i < 200; ++i) {
        diff.periodic();
        double rotationVelocityDegPerSec = rotationField.getDouble(diff);
        double rotationCommandMm =
            Math.toRadians(rotationVelocityDegPerSec)
                * DifferentialArm.kSimDifferentialArmRadiusMeters
                * 1000.0;
        peakRotationCommand = Math.max(peakRotationCommand, Math.abs(rotationCommandMm));
      }
      double maxSpoolVelocity =
          DifferentialArm.kSimMotor.freeSpeedRadPerSec
              * DifferentialArm.kSimLinearDriveRadiusMeters
              * 1000.0;
      assertEquals(
          maxSpoolVelocity,
          peakRotationCommand,
          1e-6,
          "Rotation command should not exceed motor free speed");
      assertTrue(
          diff.getLeftVelocityCommand() > 0.0 && diff.getRightVelocityCommand() < 0.0,
          "Positive rotation setpoint should command opposite spool directions");
    }
  }
}
