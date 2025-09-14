package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/** Unit tests that sanity check drive module unit conversions. */
public class ConstantsUnitTest {

  @Test
  void driveEncoderPositionFactorMatchesGeometry() {
    double expected =
        Constants.ModuleConstants.kWheelCircumferenceMeters
            / Constants.ModuleConstants.kDrivingMotorReduction;
    assertEquals(expected, Constants.ModuleConstants.kDriveEncoderPositionFactor, 1e-9);
  }

  @Test
  void driveEncoderVelocityFactorConvertsRpmToMetersPerSecond() {
    double expected = Constants.ModuleConstants.kDriveEncoderPositionFactor / 60.0;
    assertEquals(expected, Constants.ModuleConstants.kDriveEncoderVelocityFactor, 1e-9);
  }

  @Test
  void driveEncoderConversionMatchesFreeSpeed() {
    double wheelSpeed =
        Constants.NeoMotorConstants.kFreeSpeedRpm
            * Constants.ModuleConstants.kDriveEncoderVelocityFactor;
    assertEquals(Constants.ModuleConstants.kDriveWheelFreeSpeedMetersPerSecond, wheelSpeed, 1e-6);
  }

  @Test
  void driveConstantsDerivedFromModuleConstants() {
    assertEquals(
        Constants.ModuleConstants.kDriveWheelFreeSpeedMetersPerSecond,
        Constants.DriveConstants.kMaxSpeedMetersPerSecond,
        1e-9);
    double expectedMaxAngular =
        Constants.DriveConstants.kMaxSpeedMetersPerSecond
            / Math.hypot(
                Constants.DriveConstants.kWheelBase / 2.0,
                Constants.DriveConstants.kTrackWidth / 2.0);
    assertEquals(expectedMaxAngular, Constants.DriveConstants.kMaxAngularSpeed, 1e-9);
  }
}
