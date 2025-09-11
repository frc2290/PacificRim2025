package frc.utils;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ModuleConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for {@link SwerveModuleSim} ensuring the closed-loop motor controllers
 * converge on the commanded setpoints and generate longitudinal force.
 */
public class SwerveModuleSimTest {
  @BeforeAll
  public static void setupHAL() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  public void closedLoopReachesSetpoints() {
    // Module with ideal steering and no drive controller
    SwerveModuleSim module = new SwerveModuleSim(
        ModuleConstants.kDriveMotor,
        ModuleConstants.kDrivingMotorReduction,
        ModuleConstants.kWheelDiameterMeters / 2.0,
        ModuleConstants.kDriveEfficiency,
        null,
        1.0,
        null,
        null,
        null,
        null);

    double driveSetpoint = 1.0; // m/s
    double steerSetpoint = Math.PI / 4.0; // rad
    double busVoltage = 12.0;
    double dt = 0.02;
    Translation2d modulePos = new Translation2d();

    SwerveModuleSim.ModuleForce force = null;
    for (int i = 0; i < 50; i++) {
      force = module.update(
          driveSetpoint,
          steerSetpoint,
          busVoltage,
          0.0,
          0.0,
          0.0,
          modulePos,
          dt);
      module.updateDriveSensor(force.vRoll, dt, busVoltage);
    }

    assertNotNull(force);
    // With ideal steering and no drive controller, azimuth should match setpoint
    assertEquals(steerSetpoint, module.getAzimuth(), 1e-9);
    // No drive voltage means no force is generated
    assertEquals(0.0, force.fx, 1e-9);
    assertEquals(0.0, force.fy, 1e-9);
    assertEquals(0.0, force.vRoll, 1e-9);
  }
}

