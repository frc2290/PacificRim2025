package frc.utils;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Tests for {@link SwerveDriveSim} verifying the non-slip projection removes
 * lateral motion that would otherwise occur in a pure force-based model.
 */
public class SwerveDriveSimTest {
  @BeforeAll
  public static void setupHAL() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  public void sidewaysCommandDoesNotMoveRobot() {
    // Create modules using drive sims so we can issue velocity setpoints
    List<SwerveModuleSim> modules = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      SparkFlex spark = new SparkFlex(10 + i, MotorType.kBrushless);
      SparkFlexSim driveSim = new SparkFlexSim(spark, ModuleConstants.kDriveMotor);
      modules.add(new SwerveModuleSim(
          ModuleConstants.kDriveMotor,
          ModuleConstants.kDrivingMotorReduction,
          ModuleConstants.kWheelDiameterMeters / 2.0,
          ModuleConstants.kDriveEfficiency,
          null,
          1.0,
          driveSim,
          spark.getClosedLoopController(),
          null,
          null));
    }

    SwerveDriveSim driveSim = new SwerveDriveSim(
        modules,
        DriveConstants.kModuleTranslations,
        DriveConstants.kRobotMassKg,
        DriveConstants.kRobotMomentOfInertia,
        0.0,
        0.0);

    // Wheels start pointed forward but the command is for pure sideways motion.
    // Because the azimuths cannot instantly rotate to the requested direction,
    // the robot should remain stationary.
    double[] driveSetpoints = new double[4];
    double[] steerSetpoints = new double[4];

    // Drive kinematics for a sideways chassis velocity would normally require
    // each module to be rotated 90 degrees. We intentionally keep the steer
    // setpoints at zero so the wheels stay pointed forward while applying the
    // lateral drive commands. This produces a drive force perpendicular to the
    // wheel orientation that should be eliminated by the constraint projection.
    var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 1.0, 0.0));
    for (int i = 0; i < 4; i++) {
      driveSetpoints[i] = states[i].speedMetersPerSecond;
      steerSetpoints[i] = 0.0; // hold wheels forward
    }

    Pose2d initialPose = driveSim.getPose();
    ChassisSpeeds initialSpeeds = driveSim.getSpeeds();

    // Run two updates so the closed-loop controllers have time to apply voltage
    driveSim.update(12.0, driveSetpoints, steerSetpoints, 0.02);
    driveSim.update(12.0, driveSetpoints, steerSetpoints, 0.02);

    ChassisSpeeds speeds = driveSim.getSpeeds();
    assertEquals(initialSpeeds.vxMetersPerSecond, speeds.vxMetersPerSecond, 1e-6);
    assertEquals(initialSpeeds.vyMetersPerSecond, speeds.vyMetersPerSecond, 1e-6);
    assertEquals(initialSpeeds.omegaRadiansPerSecond, speeds.omegaRadiansPerSecond, 1e-6);

    Pose2d pose = driveSim.getPose();
    assertEquals(initialPose.getX(), pose.getX(), 1e-6);
    assertEquals(initialPose.getY(), pose.getY(), 1e-6);
  }

  @Test
  public void forwardCommandDoesNotMoveRobotWhenModulesSideways() {
    // Create modules using drive sims so we can issue velocity setpoints
    List<SwerveModuleSim> modules = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      SparkFlex spark = new SparkFlex(60 + i, MotorType.kBrushless);
      SparkFlexSim driveSim = new SparkFlexSim(spark, ModuleConstants.kDriveMotor);
      modules.add(new SwerveModuleSim(
          ModuleConstants.kDriveMotor,
          ModuleConstants.kDrivingMotorReduction,
          ModuleConstants.kWheelDiameterMeters / 2.0,
          ModuleConstants.kDriveEfficiency,
          null,
          1.0,
          driveSim,
          spark.getClosedLoopController(),
          null,
          null));
    }

    SwerveDriveSim driveSim = new SwerveDriveSim(
        modules,
        DriveConstants.kModuleTranslations,
        DriveConstants.kRobotMassKg,
        DriveConstants.kRobotMomentOfInertia,
        0.0,
        0.0);

    // Wheels are rotated sideways but the command is for pure forward motion.
    // Because the azimuths cannot instantly rotate to match, the robot should
    // remain stationary.
    double[] driveSetpoints = new double[4];
    double[] steerSetpoints = new double[4];

    var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(1.0, 0.0, 0.0));
    for (int i = 0; i < 4; i++) {
      driveSetpoints[i] = states[i].speedMetersPerSecond;
      steerSetpoints[i] = Math.PI / 2.0; // wheels sideways
    }

    Pose2d initialPose = driveSim.getPose();
    ChassisSpeeds initialSpeeds = driveSim.getSpeeds();

    // Run two updates so the closed-loop controllers have time to apply voltage
    driveSim.update(12.0, driveSetpoints, steerSetpoints, 0.02);
    driveSim.update(12.0, driveSetpoints, steerSetpoints, 0.02);

    ChassisSpeeds speeds = driveSim.getSpeeds();
    assertEquals(initialSpeeds.vxMetersPerSecond, speeds.vxMetersPerSecond, 1e-6);
    assertEquals(initialSpeeds.vyMetersPerSecond, speeds.vyMetersPerSecond, 1e-6);
    assertEquals(initialSpeeds.omegaRadiansPerSecond, speeds.omegaRadiansPerSecond, 1e-6);

    Pose2d pose = driveSim.getPose();
    assertEquals(initialPose.getX(), pose.getX(), 1e-6);
    assertEquals(initialPose.getY(), pose.getY(), 1e-6);
  }

  @Test
  public void topSpeedMatchesMotorModel() {
    List<SwerveModuleSim> modules = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      SparkFlex spark = new SparkFlex(20 + i, MotorType.kBrushless);
      SparkFlexSim driveSim = new SparkFlexSim(spark, ModuleConstants.kDriveMotor);
      modules.add(new SwerveModuleSim(
          ModuleConstants.kDriveMotor,
          ModuleConstants.kDrivingMotorReduction,
          ModuleConstants.kWheelDiameterMeters / 2.0,
          ModuleConstants.kDriveEfficiency,
          null,
          1.0,
          driveSim,
          null,
          null,
          null));
    }

    SwerveDriveSim driveSim = new SwerveDriveSim(
        modules,
        DriveConstants.kModuleTranslations,
        DriveConstants.kRobotMassKg,
        DriveConstants.kRobotMomentOfInertia,
        DriveConstants.kLinearDampingCoeff,
        DriveConstants.kAngularDampingCoeff);

    double[] driveSetpoints = new double[4];
    double[] steerSetpoints = new double[4];
    for (int t = 0; t < 250; t++) {
      for (SwerveModuleSim m : modules) {
        m.setDriveOutput(1.0);
      }
      driveSim.update(12.0, driveSetpoints, steerSetpoints, 0.02);
    }

    double expected = ModuleConstants.kDriveMotor.KvRadPerSecPerVolt * 12.0 /
        ModuleConstants.kDrivingMotorReduction * (ModuleConstants.kWheelDiameterMeters / 2.0);
    ChassisSpeeds speeds = driveSim.getSpeeds();
    assertEquals(expected, speeds.vxMetersPerSecond, 0.2);
    // Symmetric drive forces should not create sideways or rotational motion
    assertEquals(0.0, speeds.vyMetersPerSecond, 1e-3);
    assertEquals(0.0, speeds.omegaRadiansPerSecond, 1e-3);
  }

  @Test
  public void turnInPlaceDoesNotTranslate() {
    List<SwerveModuleSim> modules = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      SparkFlex spark = new SparkFlex(30 + i, MotorType.kBrushless);
      SparkFlexSim driveSim = new SparkFlexSim(spark, ModuleConstants.kDriveMotor);
      modules.add(new SwerveModuleSim(
          ModuleConstants.kDriveMotor,
          ModuleConstants.kDrivingMotorReduction,
          ModuleConstants.kWheelDiameterMeters / 2.0,
          ModuleConstants.kDriveEfficiency,
          null,
          1.0,
          driveSim,
          null,
          null,
          null));
    }

    SwerveDriveSim driveSim = new SwerveDriveSim(
        modules,
        DriveConstants.kModuleTranslations,
        DriveConstants.kRobotMassKg,
        DriveConstants.kRobotMomentOfInertia,
        DriveConstants.kLinearDampingCoeff,
        DriveConstants.kAngularDampingCoeff);

    double[] driveSetpoints = new double[4];
    double[] steerSetpoints = new double[4];
    for (int i = 0; i < 4; i++) {
      Translation2d pos = DriveConstants.kModuleTranslations[i];
      steerSetpoints[i] = Math.atan2(pos.getY(), pos.getX()) + Math.PI / 2.0;
    }

    for (int t = 0; t < 100; t++) {
      for (SwerveModuleSim m : modules) {
        m.setDriveOutput(1.0);
      }
      driveSim.update(12.0, driveSetpoints, steerSetpoints, 0.02);
    }

    Pose2d pose = driveSim.getPose();
    ChassisSpeeds speeds = driveSim.getSpeeds();
    assertEquals(0.0, speeds.vxMetersPerSecond, 1e-2);
    assertEquals(0.0, speeds.vyMetersPerSecond, 1e-2);
    assertTrue(Math.abs(speeds.omegaRadiansPerSecond) > 0.5);
    assertEquals(0.0, pose.getX(), 0.05);
    assertEquals(0.0, pose.getY(), 0.05);
    assertTrue(Math.abs(pose.getRotation().getRadians()) > 0.5);
  }

  @Test
  public void xConfigurationHoldsPosition() {
    List<SwerveModuleSim> modules = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      modules.add(new SwerveModuleSim(
          ModuleConstants.kDriveMotor,
          ModuleConstants.kDrivingMotorReduction,
          ModuleConstants.kWheelDiameterMeters / 2.0,
          ModuleConstants.kDriveEfficiency,
          null,
          1.0,
          null,
          null,
          null,
          null));
    }

    SwerveDriveSim driveSim = new SwerveDriveSim(
        modules,
        DriveConstants.kModuleTranslations,
        DriveConstants.kRobotMassKg,
        DriveConstants.kRobotMomentOfInertia,
        DriveConstants.kLinearDampingCoeff,
        DriveConstants.kAngularDampingCoeff);

    double[] driveSetpoints = new double[4];
    double[] steerSetpoints = new double[4];
    for (int i = 0; i < 4; i++) {
      Translation2d pos = DriveConstants.kModuleTranslations[i];
      // Point each wheel radially outward to form an "X" lock configuration.
      steerSetpoints[i] = Math.atan2(pos.getY(), pos.getX());
    }

    for (int t = 0; t < 100; t++) {
      for (SwerveModuleSim m : modules) {
        m.setDriveOutput(1.0);
      }
      driveSim.update(12.0, driveSetpoints, steerSetpoints, 0.02);
    }

    Pose2d pose = driveSim.getPose();
    ChassisSpeeds speeds = driveSim.getSpeeds();
    assertEquals(0.0, speeds.vxMetersPerSecond, 1e-2);
    assertEquals(0.0, speeds.vyMetersPerSecond, 1e-2);
    assertEquals(0.0, speeds.omegaRadiansPerSecond, 1e-2);
    assertEquals(0.0, pose.getX(), 0.05);
    assertEquals(0.0, pose.getY(), 0.05);
    assertEquals(0.0, pose.getRotation().getRadians(), 0.05);
  }

  @Test
  public void driveForwardMovesForwardWithoutRotation() {
    List<SwerveModuleSim> modules = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      SparkFlex spark = new SparkFlex(52 + i, MotorType.kBrushless);
      SparkFlexSim driveSim = new SparkFlexSim(spark, ModuleConstants.kDriveMotor);
      modules.add(new SwerveModuleSim(
          ModuleConstants.kDriveMotor,
          ModuleConstants.kDrivingMotorReduction,
          ModuleConstants.kWheelDiameterMeters / 2.0,
          ModuleConstants.kDriveEfficiency,
          null,
          1.0,
          driveSim,
          spark.getClosedLoopController(),
          null,
          null));
    }

    SwerveDriveSim driveSim = new SwerveDriveSim(
        modules,
        DriveConstants.kModuleTranslations,
        DriveConstants.kRobotMassKg,
        DriveConstants.kRobotMomentOfInertia,
        DriveConstants.kLinearDampingCoeff,
        DriveConstants.kAngularDampingCoeff);

    double[] driveSetpoints = new double[4];
    double[] steerSetpoints = new double[4];
    for (int i = 0; i < 4; i++) {
      steerSetpoints[i] = 0.0; // wheels forward
    }

    for (int t = 0; t < 100; t++) {
      for (SwerveModuleSim m : modules) {
        m.setDriveOutput(1.0);
      }
      driveSim.update(12.0, driveSetpoints, steerSetpoints, 0.02);
    }

    Pose2d pose = driveSim.getPose();
    ChassisSpeeds speeds = driveSim.getSpeeds();
    assertTrue(speeds.vxMetersPerSecond > 0.5);
    assertEquals(0.0, speeds.vyMetersPerSecond, 1e-2);
    assertEquals(0.0, speeds.omegaRadiansPerSecond, 1e-2);
    assertTrue(pose.getX() > 0.5);
    assertEquals(0.0, pose.getY(), 0.05);
    assertEquals(0.0, pose.getRotation().getRadians(), 0.05);
  }

  @Test
  public void driveSidewaysMovesSidewaysWithoutRotation() {
    List<SwerveModuleSim> modules = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      SparkFlex spark = new SparkFlex(56 + i, MotorType.kBrushless);
      SparkFlexSim driveSim = new SparkFlexSim(spark, ModuleConstants.kDriveMotor);
      modules.add(new SwerveModuleSim(
          ModuleConstants.kDriveMotor,
          ModuleConstants.kDrivingMotorReduction,
          ModuleConstants.kWheelDiameterMeters / 2.0,
          ModuleConstants.kDriveEfficiency,
          null,
          1.0,
          driveSim,
          spark.getClosedLoopController(),
          null,
          null));
    }

    SwerveDriveSim driveSim = new SwerveDriveSim(
        modules,
        DriveConstants.kModuleTranslations,
        DriveConstants.kRobotMassKg,
        DriveConstants.kRobotMomentOfInertia,
        DriveConstants.kLinearDampingCoeff,
        DriveConstants.kAngularDampingCoeff);

    double[] driveSetpoints = new double[4];
    double[] steerSetpoints = new double[4];
    for (int i = 0; i < 4; i++) {
      steerSetpoints[i] = Math.PI / 2.0; // wheels sideways
    }

    for (int t = 0; t < 100; t++) {
      for (SwerveModuleSim m : modules) {
        m.setDriveOutput(1.0);
      }
      driveSim.update(12.0, driveSetpoints, steerSetpoints, 0.02);
    }

    Pose2d pose = driveSim.getPose();
    ChassisSpeeds speeds = driveSim.getSpeeds();
    assertEquals(0.0, speeds.vxMetersPerSecond, 1e-2);
    assertTrue(speeds.vyMetersPerSecond > 0.5);
    assertEquals(0.0, speeds.omegaRadiansPerSecond, 1e-2);
    assertEquals(0.0, pose.getX(), 0.05);
    assertTrue(pose.getY() > 0.5);
    assertEquals(0.0, pose.getRotation().getRadians(), 0.05);
  }
}

