package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class DriveSubsystemTest {

  @Mock private MAXSwerveModule frontLeft;
  @Mock private MAXSwerveModule frontRight;
  @Mock private MAXSwerveModule rearLeft;
  @Mock private MAXSwerveModule rearRight;
  @Mock private AHRS gyro;

  private DriveSubsystem newDrive() {
    return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro);
  }

  @Test
  void setSlowAndRegularSpeedSwitchesMode() {
    try (DriveSubsystem drive = newDrive()) {
      drive.setSlowSpeed();
      assertTrue(drive.isSlowSpeed());

      drive.setRegularSpeed();
      assertFalse(drive.isSlowSpeed());
    }
  }

  @Test
  void getCurrentDrawSumsAllModules() {
    try (DriveSubsystem drive = newDrive()) {
      when(frontLeft.getCurrentDraw()).thenReturn(1.0);
      when(frontRight.getCurrentDraw()).thenReturn(2.0);
      when(rearLeft.getCurrentDraw()).thenReturn(3.0);
      when(rearRight.getCurrentDraw()).thenReturn(4.0);

      double total = drive.getCurrentDraw();

      assertEquals(10.0, total, 1e-9);
    }
  }

  @Test
  void getModuleCurrentsReturnsPerModuleValues() {
    try (DriveSubsystem drive = newDrive()) {
      when(frontLeft.getCurrentDraw()).thenReturn(1.0);
      when(frontRight.getCurrentDraw()).thenReturn(2.0);
      when(rearLeft.getCurrentDraw()).thenReturn(3.0);
      when(rearRight.getCurrentDraw()).thenReturn(4.0);

      assertArrayEquals(new double[] {1.0, 2.0, 3.0, 4.0}, drive.getModuleCurrents(), 1e-9);
    }
  }

  @Test
  void driveSetsModuleStates() {
    try (DriveSubsystem drive = newDrive()) {
      drive.drive(1.0, 0.0, 0.0, false);

      SwerveModuleState[] expected =
          DriveConstants.kDriveKinematics.toSwerveModuleStates(
              new ChassisSpeeds(DriveConstants.kMaxSpeedMetersPerSecond, 0.0, 0.0));
      SwerveDriveKinematics.desaturateWheelSpeeds(
          expected, DriveConstants.kMaxSpeedMetersPerSecond);

      ArgumentCaptor<SwerveModuleState> flCaptor = ArgumentCaptor.forClass(SwerveModuleState.class);
      ArgumentCaptor<SwerveModuleState> frCaptor = ArgumentCaptor.forClass(SwerveModuleState.class);
      ArgumentCaptor<SwerveModuleState> rlCaptor = ArgumentCaptor.forClass(SwerveModuleState.class);
      ArgumentCaptor<SwerveModuleState> rrCaptor = ArgumentCaptor.forClass(SwerveModuleState.class);

      verify(frontLeft).setDesiredState(flCaptor.capture());
      verify(frontRight).setDesiredState(frCaptor.capture());
      verify(rearLeft).setDesiredState(rlCaptor.capture());
      verify(rearRight).setDesiredState(rrCaptor.capture());

      assertStateEquals(expected[0], flCaptor.getValue());
      assertStateEquals(expected[1], frCaptor.getValue());
      assertStateEquals(expected[2], rlCaptor.getValue());
      assertStateEquals(expected[3], rrCaptor.getValue());
    }
  }

  @Test
  void setXStopsModules() {
    try (DriveSubsystem drive = newDrive()) {
      drive.setX();

      ArgumentCaptor<SwerveModuleState> flCaptor = ArgumentCaptor.forClass(SwerveModuleState.class);
      ArgumentCaptor<SwerveModuleState> frCaptor = ArgumentCaptor.forClass(SwerveModuleState.class);
      ArgumentCaptor<SwerveModuleState> rlCaptor = ArgumentCaptor.forClass(SwerveModuleState.class);
      ArgumentCaptor<SwerveModuleState> rrCaptor = ArgumentCaptor.forClass(SwerveModuleState.class);

      verify(frontLeft).setDesiredState(flCaptor.capture());
      verify(frontRight).setDesiredState(frCaptor.capture());
      verify(rearLeft).setDesiredState(rlCaptor.capture());
      verify(rearRight).setDesiredState(rrCaptor.capture());

      assertStateEquals(
          new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)), flCaptor.getValue());
      assertStateEquals(
          new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)), frCaptor.getValue());
      assertStateEquals(
          new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)), rlCaptor.getValue());
      assertStateEquals(
          new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)), rrCaptor.getValue());
    }
  }

  @Test
  void zeroHeadingResetsGyro() {
    try (DriveSubsystem drive = newDrive()) {
      drive.zeroHeading();
      verify(gyro).reset();
    }
  }

  @Test
  void getHeadingReturnsYaw() {
    try (DriveSubsystem drive = newDrive()) {
      when(gyro.getYaw()).thenReturn(30f);
      assertEquals(30.0, drive.getHeading(), 1e-9);
    }
  }

  private static void assertStateEquals(SwerveModuleState expected, SwerveModuleState actual) {
    assertEquals(expected.speedMetersPerSecond, actual.speedMetersPerSecond, 1e-9);
    assertEquals(expected.angle.getRadians(), actual.angle.getRadians(), 1e-9);
  }
}
