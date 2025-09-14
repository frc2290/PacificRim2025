package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.atLeastOnce;
import static org.mockito.Mockito.doNothing;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

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
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveModuleSim;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentMatchers;

public class DriveSubsystemSimTest {
  @BeforeAll
  public static void setupHal() {
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
    AHRS gyro = mock(AHRS.class);
    when(gyro.getAngle()).thenReturn(0.0);
    when(gyro.getYaw()).thenReturn(0.0f);
    when(gyro.getRate()).thenReturn(0.0);
    when(gyro.getRotation2d()).thenReturn(new Rotation2d());
    doNothing().when(gyro).setAngleAdjustment(ArgumentMatchers.anyDouble());
    return new DriveSubsystem(modules[0], modules[1], modules[2], modules[3], gyro);
  }

  @Test
  public void positiveXErrorCommandsForwardMotion() {
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
  public void negativeXErrorCommandsBackwardMotion() {
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
  public void positiveYErrorCommandsLeftMotion() {
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
  public void negativeYErrorCommandsRightMotion() {
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
  public void positiveRotationErrorCommandsCCWTurn() {
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
  public void driveChassisSpeedsSetsModuleStates() {
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
  public void simulationUsesDesiredStateForModuleSim() {
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
}
