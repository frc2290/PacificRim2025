package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/** Unit tests that sanity check drive module unit conversions. */
public class ConstantsUnitTest {
  @Test
  void driveEncoderConversionMatchesFreeSpeed() {
    double wheelSpeed =
        Constants.NeoMotorConstants.kFreeSpeedRpm * Constants.ModuleConstants.kDriveEncoderVelocityFactor;
    assertEquals(Constants.ModuleConstants.kDriveWheelFreeSpeedMetersPerSecond, wheelSpeed, 1e-6);
  }
}

