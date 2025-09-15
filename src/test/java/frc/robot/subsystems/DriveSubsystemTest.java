package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.DriveConstants;
import frc.utils.PoseEstimatorSubsystem;
import frc.utils.SwerveModuleSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
class DriveSubsystemTest {

  @Mock private MAXSwerveModule frontLeft;
  @Mock private MAXSwerveModule frontRight;
  @Mock private MAXSwerveModule rearLeft;
  @Mock private MAXSwerveModule rearRight;
  @Mock private AHRS gyro;

  private DriveSubsystem newDrive() {
    return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro);
  }

  @BeforeAll
  static void setupHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  private MAXSwerveModule newModule(double offset) {
    SparkFlex driveSpark = mock(SparkFlex.class);
    SparkMax turnSpark = mock(SparkMax.class);
    RelativeEncoder driveEnc = mock(RelativeEncoder.class);
    AbsoluteEncoder turnEnc = mock(AbsoluteEncoder.class);
    SparkClosedLoopController driveCtrl = mock(SparkClosedLoopController.class);
    SparkClosedLoopController turnCtrl = mock(SparkClosedLoopController.class);
    when(turnEnc.getPosition()).thenReturn(0.0);
    return new MAXSwerveModule(
        driveSpark, turnSpark, driveEnc, turnEnc, driveCtrl, turnCtrl, offset);
  }

  private ChassisSpeeds commandedFromModules(MAXSwerveModule[] modules) {
    double[] offsets = {
      DriveConstants.kFrontLeftChassisAngularOffset,
      DriveConstants.kFrontRightChassisAngularOffset,
      DriveConstants.kBackLeftChassisAngularOffset,
      DriveConstants.kBackRightChassisAngularOffset
    };
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      SwerveModuleState desired = modules[i].getDesiredState();
      states[i] =
          new SwerveModuleState(
              desired.speedMetersPerSecond,
              desired.angle.minus(Rotation2d.fromRadians(offsets[i])));
    }
    return DriveConstants.kDriveKinematics.toChassisSpeeds(states);
  }

  private DriveSubsystem newSubsystem(MAXSwerveModule[] modules) {
    AHRS g = mock(AHRS.class);
    when(g.getAngle()).thenReturn(0.0);
    when(g.getYaw()).thenReturn(0.0f);
    when(g.getRate()).thenReturn(0.0);
    when(g.getRotation2d()).thenReturn(new Rotation2d());
    doNothing().when(g).setAngleAdjustment(anyDouble());
    return new DriveSubsystem(modules[0], modules[1], modules[2], modules[3], g);
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

  @Test
  void positiveXErrorCommandsForwardMotion() {
    MAXSwerveModule[] modules = {
      newModule(DriveConstants.kFrontLeftChassisAngularOffset),
      newModule(DriveConstants.kFrontRightChassisAngularOffset),
      newModule(DriveConstants.kBackLeftChassisAngularOffset),
      newModule(DriveConstants.kBackRightChassisAngularOffset)
    };
    try (DriveSubsystem drive = newSubsystem(modules)) {
      Pose2d target = new Pose2d(1.0, 0.0, new Rotation2d());
      Pose2d current = new Pose2d();
      ChassisSpeeds speeds =
          new ChassisSpeeds(
              target.getX() - current.getX(),
              target.getY() - current.getY(),
              target.getRotation().minus(current.getRotation()).getRadians());
      drive.driveChassisSpeeds(speeds);
      ChassisSpeeds commanded = commandedFromModules(modules);
      assertTrue(commanded.vxMetersPerSecond > 0.0);
      assertEquals(0.0, commanded.vyMetersPerSecond, 1e-6);
      assertEquals(0.0, commanded.omegaRadiansPerSecond, 1e-6);
    }
  }

  @Test
  void negativeXErrorCommandsBackwardMotion() {
    MAXSwerveModule[] modules = {
      newModule(DriveConstants.kFrontLeftChassisAngularOffset),
      newModule(DriveConstants.kFrontRightChassisAngularOffset),
      newModule(DriveConstants.kBackLeftChassisAngularOffset),
      newModule(DriveConstants.kBackRightChassisAngularOffset)
    };
    try (DriveSubsystem drive = newSubsystem(modules)) {
      Pose2d target = new Pose2d(-1.0, 0.0, new Rotation2d());
      Pose2d current = new Pose2d();
      ChassisSpeeds speeds =
          new ChassisSpeeds(
              target.getX() - current.getX(),
              target.getY() - current.getY(),
              target.getRotation().minus(current.getRotation()).getRadians());
      drive.driveChassisSpeeds(speeds);
      ChassisSpeeds commanded = commandedFromModules(modules);
      assertTrue(commanded.vxMetersPerSecond < 0.0);
      assertEquals(0.0, commanded.vyMetersPerSecond, 1e-6);
      assertEquals(0.0, commanded.omegaRadiansPerSecond, 1e-6);
    }
  }

  @Test
  void positiveYErrorCommandsLeftMotion() {
    MAXSwerveModule[] modules = {
      newModule(DriveConstants.kFrontLeftChassisAngularOffset),
      newModule(DriveConstants.kFrontRightChassisAngularOffset),
      newModule(DriveConstants.kBackLeftChassisAngularOffset),
      newModule(DriveConstants.kBackRightChassisAngularOffset)
    };
    try (DriveSubsystem drive = newSubsystem(modules)) {
      Pose2d target = new Pose2d(0.0, 1.0, new Rotation2d());
      Pose2d current = new Pose2d();
      ChassisSpeeds speeds =
          new ChassisSpeeds(
              target.getX() - current.getX(),
              target.getY() - current.getY(),
              target.getRotation().minus(current.getRotation()).getRadians());
      drive.driveChassisSpeeds(speeds);
      ChassisSpeeds commanded = commandedFromModules(modules);
      assertTrue(commanded.vyMetersPerSecond > 0.0);
      assertEquals(0.0, commanded.vxMetersPerSecond, 1e-6);
      assertEquals(0.0, commanded.omegaRadiansPerSecond, 1e-6);
    }
  }

  @Test
  void negativeYErrorCommandsRightMotion() {
    MAXSwerveModule[] modules = {
      newModule(DriveConstants.kFrontLeftChassisAngularOffset),
      newModule(DriveConstants.kFrontRightChassisAngularOffset),
      newModule(DriveConstants.kBackLeftChassisAngularOffset),
      newModule(DriveConstants.kBackRightChassisAngularOffset)
    };
    try (DriveSubsystem drive = newSubsystem(modules)) {
      Pose2d target = new Pose2d(0.0, -1.0, new Rotation2d());
      Pose2d current = new Pose2d();
      ChassisSpeeds speeds =
          new ChassisSpeeds(
              target.getX() - current.getX(),
              target.getY() - current.getY(),
              target.getRotation().minus(current.getRotation()).getRadians());
      drive.driveChassisSpeeds(speeds);
      ChassisSpeeds commanded = commandedFromModules(modules);
      assertTrue(commanded.vyMetersPerSecond < 0.0);
      assertEquals(0.0, commanded.vxMetersPerSecond, 1e-6);
      assertEquals(0.0, commanded.omegaRadiansPerSecond, 1e-6);
    }
  }

  @Test
  void positiveRotationErrorCommandsCCWTurn() {
    MAXSwerveModule[] modules = {
      newModule(DriveConstants.kFrontLeftChassisAngularOffset),
      newModule(DriveConstants.kFrontRightChassisAngularOffset),
      newModule(DriveConstants.kBackLeftChassisAngularOffset),
      newModule(DriveConstants.kBackRightChassisAngularOffset)
    };
    try (DriveSubsystem drive = newSubsystem(modules)) {
      Pose2d target = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90));
      Pose2d current = new Pose2d();
      ChassisSpeeds speeds =
          new ChassisSpeeds(
              0.0, 0.0, target.getRotation().minus(current.getRotation()).getRadians());
      drive.driveChassisSpeeds(speeds);
      ChassisSpeeds commanded = commandedFromModules(modules);
      assertTrue(commanded.omegaRadiansPerSecond > 0.0);
      assertEquals(0.0, commanded.vxMetersPerSecond, 1e-6);
      assertEquals(0.0, commanded.vyMetersPerSecond, 1e-6);
    }
  }

  @Test
  void driveChassisSpeedsSetsModuleStates() {
    MAXSwerveModule[] modules = {
      newModule(DriveConstants.kFrontLeftChassisAngularOffset),
      newModule(DriveConstants.kFrontRightChassisAngularOffset),
      newModule(DriveConstants.kBackLeftChassisAngularOffset),
      newModule(DriveConstants.kBackRightChassisAngularOffset)
    };
    try (DriveSubsystem drive = newSubsystem(modules)) {
      ChassisSpeeds speeds = new ChassisSpeeds(1.0, 0.5, 0.25);
      drive.driveChassisSpeeds(speeds);

      SwerveModuleState[] expected = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(
          expected, DriveConstants.kMaxSpeedMetersPerSecond);

      double[] offsets = {
        DriveConstants.kFrontLeftChassisAngularOffset,
        DriveConstants.kFrontRightChassisAngularOffset,
        DriveConstants.kBackLeftChassisAngularOffset,
        DriveConstants.kBackRightChassisAngularOffset
      };

      for (int i = 0; i < modules.length; i++) {
        SwerveModuleState corrected =
            new SwerveModuleState(
                expected[i].speedMetersPerSecond,
                expected[i].angle.plus(Rotation2d.fromRadians(offsets[i])));
        corrected.optimize(new Rotation2d());

        SwerveModuleState desired = modules[i].getDesiredState();
        assertEquals(corrected.speedMetersPerSecond, desired.speedMetersPerSecond, 1e-6);
        assertEquals(corrected.angle.getRadians(), desired.angle.getRadians(), 1e-6);
      }
    }
  }

  @Test
  void simulationUsesDesiredStateForModuleSim() {
    MAXSwerveModule[] modules = {
      newModule(DriveConstants.kFrontLeftChassisAngularOffset),
      newModule(DriveConstants.kFrontRightChassisAngularOffset),
      newModule(DriveConstants.kBackLeftChassisAngularOffset),
      newModule(DriveConstants.kBackRightChassisAngularOffset)
    };
    try (DriveSubsystem drive = newSubsystem(modules)) {
      SwerveModuleState[] commanded = {
        new SwerveModuleState(1.0, new Rotation2d()),
        new SwerveModuleState(-2.0, new Rotation2d()),
        new SwerveModuleState(3.0, new Rotation2d()),
        new SwerveModuleState(-4.0, new Rotation2d())
      };

      drive.setModuleStates(commanded);

      try {
        var field = DriveSubsystem.class.getDeclaredField("m_moduleSims");
        field.setAccessible(true);
        SwerveModuleSim[] sims = (SwerveModuleSim[]) field.get(drive);
        for (int i = 0; i < sims.length; i++) {
          sims[i] = spy(sims[i]);
        }
        field.set(drive, sims);

        drive.updateSimState(0.02);

        double[] offsets = {
          DriveConstants.kFrontLeftChassisAngularOffset,
          DriveConstants.kFrontRightChassisAngularOffset,
          DriveConstants.kBackLeftChassisAngularOffset,
          DriveConstants.kBackRightChassisAngularOffset
        };
        for (int i = 0; i < sims.length; i++) {
          double expectedAngle = modules[i].getDesiredState().angle.getRadians() - offsets[i];
          double expectedSpeed = modules[i].getDesiredState().speedMetersPerSecond;
          verify(sims[i], atLeastOnce())
              .update(
                  eq(expectedSpeed),
                  eq(expectedAngle),
                  anyDouble(),
                  anyDouble(),
                  anyDouble(),
                  anyDouble(),
                  any(),
                  anyDouble());
        }
      } catch (ReflectiveOperationException e) {
        throw new RuntimeException(e);
      }
    }
  }

  @Test
  void estimatorTracksGroundTruth() {
    RoboRioSim.setVInVoltage(12.0);
    try (DriveSubsystem drive = new DriveSubsystem()) {
      PoseEstimatorSubsystem estimator =
          new PoseEstimatorSubsystem(
              drive::newHeading, drive::getModulePositions, drive::getModuleStates);

      SwerveModuleState[] states =
          DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(1.0, 0.0, 0.0));
      drive.setModuleStates(states);

      for (int i = 0; i < 50; i++) {
        Timer.delay(0.02);
        drive.simulationPeriodic();
        estimator.periodic();
      }

      Pose2d truth = drive.getSimPose();
      Pose2d estimated = estimator.getCurrentPose();
      assertEquals(truth.getX(), estimated.getX(), 0.05);
      assertEquals(truth.getY(), estimated.getY(), 0.05);
      assertEquals(truth.getRotation().getRadians(), estimated.getRotation().getRadians(), 0.05);
    }
  }

  private static void assertStateEquals(SwerveModuleState expected, SwerveModuleState actual) {
    assertEquals(expected.speedMetersPerSecond, actual.speedMetersPerSecond, 1e-9);
    assertEquals(expected.angle.getRadians(), actual.angle.getRadians(), 1e-9);
  }
}
