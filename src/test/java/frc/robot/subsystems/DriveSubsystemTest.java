package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.io.DriveIO;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class DriveSubsystemTest {
  private static class MockDriveIO implements DriveIO {
    public ChassisSpeeds lastSpeeds = new ChassisSpeeds();
    public boolean zeroed = false;

    @Override
    public void setChassisSpeeds(ChassisSpeeds speeds) {
      lastSpeeds = speeds;
    }

    @Override
    public void zeroGyro() {
      zeroed = true;
    }

    @Override
    public Rotation2d getGyroAngle() {
      return new Rotation2d();
    }

    @Override
    public SwerveModulePosition[] getModulePositions() {
      return new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
      return new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };
    }

    @Override
    public double[] getModuleCurrents() {
      return new double[] {1.0, 2.0, 3.0, 4.0};
    }
  }

  @BeforeAll
  static void init() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void setSlowAndRegularSpeedSwitchesMode() {
    var io = new MockDriveIO();
    DriveSubsystem drive = new DriveSubsystem(io);
    drive.setSlowSpeed();
    assertTrue(drive.isSlowSpeed());
    drive.setRegularSpeed();
    assertFalse(drive.isSlowSpeed());
  }

  @Test
  void driveSetsChassisSpeedsOnIO() {
    var io = new MockDriveIO();
    DriveSubsystem drive = new DriveSubsystem(io);
    drive.drive(1.0, 0.0, 0.0, false);
    assertEquals(DriveConstants.kMaxSpeedMetersPerSecond, io.lastSpeeds.vxMetersPerSecond, 1e-9);
  }

  @Test
  void zeroHeadingDelegatesToIO() {
    var io = new MockDriveIO();
    DriveSubsystem drive = new DriveSubsystem(io);
    drive.zeroHeading();
    assertTrue(io.zeroed);
  }

  @Test
  void getCurrentDrawSumsModuleCurrents() {
    var io = new MockDriveIO();
    DriveSubsystem drive = new DriveSubsystem(io);
    assertEquals(10.0, drive.getCurrentDraw(), 1e-9);
  }
}
