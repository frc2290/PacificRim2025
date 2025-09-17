// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import java.util.List;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final boolean debugMode = true;

  public static final class Climber {
    public static final int kLeftClimberMotorId = 8;
    public static final int kRightClimberMotorId = 81;

    public static final double kPositionConversion =
        1.0; // ((7/34) * (15/31) * (0.0538 * Math.PI)); //
    // Meters Position
    public static final double kVelocityConversion =
        1.0; // ((7/34) * (15/31) * (0.0538 * Math.PI)); //
    // Meters Position

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kKG = 0.0;

    public static final double transportSetpoint = 0;
    public static final double climberOutSetpoint = -210;
    public static final double climberInSetpoint = -85;

    // Simulation parameters
    // Gear reduction between motor and climber arm
    public static final double kSimGearing = 250.0;
    // Approximate length of the climber arm (meters)
    public static final double kSimArmLengthMeters = 0.25;
    // Mass of the arm when deploying (kg)
    public static final double kSimDeployMassKg = 5.0;
    // Mass of the arm plus robot when retracting (kg)
    public static final double kSimRetractMassKg = 55.0;
  }

  public static final class Elevator {
    public static final int kLeftElevatorMotorId = 50;
    public static final int kRightElevatorMotorId = 51;

    public static final double kPositionConversion =
        0.01683762514; // ((7/34) * (15/31) * (0.0538 * Math.PI)); //
    // Meters Position
    public static final double kVelocityConversion =
        0.00028062708; // ((7/34) * (15/31) * (0.0538 * Math.PI)); //
    // Meters Position

    public static final double kP = 0.125;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kKG = 0.0;

    public static final double kS = 0.8842;
    public static final double kV = 5.8979;
    public static final double kA = 0.1156;
    public static final double kG = 0.1987;

    public static final double transportSetpoint = 0.3;
    public static final double intakeSetpoint = 0.1;

    // Simulation parameters
    // Gear reduction from motor to drum: (7/34) * (15/31)
    public static final double kSimGearing = (7.0 / 34.0) * (15.0 / 31.0);
    public static final double kSimDrumRadiusMeters = 0.0538; // m
    public static final double kSimCarriageMassKg = 4.0; // kg
    public static final double kSimMinHeightMeters = 0.0; // m
    public static final double kSimMaxHeightMeters = 1.3; // m
  }

  public static final class Manipulator {
    public static final int kManipulatorMotorId = 7;
    // Simulation parameters
    public static final double kSimGearing = 1.0; // ratio
    public static final double kSimMOI = 5e-4; // kg*m^2
  }

  public static final class DifferentialArm {
    // Motors
    public static final int kLeftMotorId = 60;
    public static final int kRightMotorId = 61;
    // configuration
    public static final double voltageComp = 0;
    public static final int currentStallLim = 0;
    public static final int currentFreeLim = 0;

    public static final double v_kp = 0.00009;
    public static final double v_ki = (0.002 * v_kp);
    public static final double v_kd = 0;
    public static final double v_KG = 0;

    public static final double transportExtensionSetpoint = 80;
    public static final double transportRotationSetpoint = 230;
    public static final double intakeExtensionSetpoint = 10;
    public static final double intakeRotationSetpoint = 230;

    public static final int kLaserCanId = 5;

    public static final double kEncoderPositionFactor = 34.2857;
    public static final double kEncoderVelocityFactor = 0.5714;

    public static final double[][] l4RotationData = {
      {120, 235},
      {200, 240},
      {330, 230},
      {420, 230}
    };

    public static final double[][] l4ExtensionData = {
      {120, 140},
      {200, 250},
      {330, 260},
      {420, 260}
    };

    public static final double[][] l2_3RotationData = {
      {120, 230},
      {200, 230},
      {330, 222},
      {420, 218}
    };

    public static final double[][] l2_3ExtensionData = {
      {120, 80},
      {200, 170},
      {330, 170},
      {420, 225}
    };

    // Differential arm simulation parameters (best-fit values)
    public static final double kSimExtensionMassKg = 1.1292; // kg
    public static final double kSimRotationMassKg = 2.0412; // kg
    public static final double kSimRotationInertiaKgM2 = 0.0468219; // kg*m^2
    public static final double kSimComOffsetMeters = 0.029518; // m
    public static final double kSimExtensionInclinationRads = 0.523599; // rad
    public static final double kSimGravity = 9.81; // m/s^2
    public static final double kSimExtensionViscousDamping = 0.530938; // N*s/m
    public static final double kSimExtensionCoulombFriction = 65.9326; // N
    public static final double kSimRotationViscousDamping = 0.375174; // N*m*s/rad
    public static final double kSimRotationCoulombFriction = 0.31433; // N*m
    public static final double kSimLinearDriveRadiusMeters = 0.00545674; // m
    public static final double kSimDifferentialArmRadiusMeters = 0.031831; // m
    public static final double kSimSensorOffsetRads = 2.13296; // rad
    public static final double kSimMotorRotorInertia = 6.52157e-7; // kg*m^2
    public static final double kSimMinExtensionMeters = 0.0; // m
    public static final double kSimMaxExtensionMeters = 0.5; // m
    public static final double kSimMinThetaRads = 0.0; // rad
    public static final double kSimMaxThetaRads = 2 * Math.PI; // rad
    public static final double kSimStartingExtensionMeters = 0.0; // m
    public static final double kSimStartingThetaRads = 0.0; // rad
  }

  public static final class DriveConstants {
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final Translation2d[] kModuleTranslations = {
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    };
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(kModuleTranslations);

    // Driving Parameters - the following limits are derived from the physical
    // characteristics of the module and robot geometry.
    public static final double kMaxSpeedMetersPerSecond =
        ModuleConstants.kDriveWheelFreeSpeedMetersPerSecond;
    public static final double kMaxAngularSpeed =
        kMaxSpeedMetersPerSecond
            / Math.hypot(kWheelBase / 2.0, kTrackWidth / 2.0); // radians per second

    // Physical properties used for simulation
    public static final double kRobotMassKg = 50.0;
    public static final double kLinearDampingCoeff = 0.2;
    public static final double kAngularDampingCoeff = 0.05;
    // Blend factor for softening non-holonomic constraints in simulation (1.0 = rigid)
    public static final double kConstraintBeta = 0.9;
    public static final double kRobotMomentOfInertia =
        kRobotMassKg * (kWheelBase * kWheelBase + kTrackWidth * kTrackWidth) / 12.0;

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = (-Math.PI / 2);
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = (Math.PI / 2);

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 42;
    public static final int kRearLeftDrivingCanId = 40;
    public static final int kFrontRightDrivingCanId = 44;
    public static final int kRearRightDrivingCanId = 46;

    public static final int kFrontLeftTurningCanId = 43;
    public static final int kRearLeftTurningCanId = 41;
    public static final int kFrontRightTurningCanId = 45;
    public static final int kRearRightTurningCanId = 47;

    public static final boolean kGyroReversed = false;

    public static final DriveTrainSimulationConfig mapleSimConfig =
        DriveTrainSimulationConfig.Default()
            .withCustomModuleTranslations(kModuleTranslations)
            .withRobotMass(Kilogram.of(kRobotMassKg))
            .withGyro(COTS.ofNav2X())
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    ModuleConstants.kDriveMotor,
                    ModuleConstants.kSteerMotor,
                    ModuleConstants.kDrivingMotorReduction,
                    ModuleConstants.kSteerReduction,
                    Volts.of(0.1),
                    Volts.of(0.1),
                    Meters.of(ModuleConstants.kWheelDiameterMeters / 2.0),
                    KilogramSquareMeters.of(0.02),
                    1.2));
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    // Wheel size and gearing
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear,
    // and 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15.0);

    // Encoder conversion factors: raw motor rotations -> meters
    public static final double kDriveEncoderPositionFactor =
        kWheelCircumferenceMeters / kDrivingMotorReduction; // meters per motor rotation
    // The Spark Flex reports velocity in RPM, so convert motor RPM to linear wheel
    // speed using 60 seconds per minute.
    public static final double kDriveEncoderVelocityFactor =
        kDriveEncoderPositionFactor / 60.0; // meters per second per RPM

    // Free speed of the drive wheel in meters per second
    public static final double kDriveWheelFreeSpeedMetersPerSecond =
        kDrivingMotorFreeSpeedRps * kDriveEncoderPositionFactor;

    // Steering and efficiency parameters
    public static final double kSteerReduction = 9424.0 / 203.0;
    public static final double kDriveEfficiency = 0.92;

    // Current limit for drive motors in simulation (amps)
    public static final int kDriveCurrentLimitAmps = 50;

    // Motor models used for simulation
    public static final DCMotor kDriveMotor = DCMotor.getNeoVortex(1);
    public static final DCMotor kSteerMotor = DCMotor.getNeo550(1);
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    /** Default translation scalar for DriveTestAuto (1.0 = full speed). */
    public static final double kDriveTestDefaultTranslationScalar = 0.1;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    public static final PathConstraints kPathConstraints =
        new PathConstraints(
            kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared,
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularSpeedRadiansPerSecondSquared);

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  /** Radius around a location that counts as "inside" an intake zone in simulation (m). */
  public static final double SIM_INTAKE_TOLERANCE_METERS = 0.5;

  public static final class VisionConstants {
    public static final double CAMERA_HEIGHT_METERS = 0.9144;
    public static final double CAMERA_PITCH_RADIANS = degreesToRadians(45);
    public static final double TARGET_HEIGHT_METERS = 1.4351;

    /**
     * Physical location of the apriltag camera on the robot, relative to the center of the robot.
     */
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT =
        new Transform3d(
            new Translation3d(0.239191, -0.344616, 0.324842),
            new Rotation3d(0.0, 0, degreesToRadians(30)));

    public static final Transform3d APRILTAG_CAMERA2_TO_ROBOT =
        new Transform3d(
            new Translation3d(0.0015, -0.3279, (0.9473 - 0.102)),
            new Rotation3d(degreesToRadians(180), degreesToRadians(-45), degreesToRadians(-200)));

    // Simulated camera parameters
    public static final int CAMERA_RESOLUTION_WIDTH = 960; // pixels
    public static final int CAMERA_RESOLUTION_HEIGHT = 720; // pixels
    public static final double CAMERA_FOV_DEGREES = 90; // diagonal field of view
    public static final double CAMERA_FPS = 20.0; // frames per second
    public static final double CAMERA_AVG_LATENCY_MS = 30.0; // milliseconds
    public static final double CAMERA_LATENCY_STDDEV_MS = 5.0; // milliseconds

    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose
    // to the opposite alliance
    public static final Pose2d FLIPPING_POSE =
        new Pose2d(
            new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS), new Rotation2d(Math.PI));

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

    /*
     * April Tag IDs:
     * -Red Coral stations: left 1, right 2
     * -Red Processor 3
     * -Red side of barge: blue barge 4, red barge 5
     * -Red reef: FL 6, F 7, FR 8, BR 9, B 10, BL 11
     * -Blue Coral Stations: right 12, left 13
     * -Blue side of barge: blue barge 14, red barge 15
     * -Blue processor: 16
     * -Blue reef: FR 17, F 18, FL 19, BL 20, B 21, BR 22
     */

    public static final double inToM = 1 / 39.37;
    public final double mToIn = 39.37;

    // All Variables in Blue coordinate system
    // PHANTOM LAKES
    // public static final List<Pose2d> leftBranches = List.of(
    //         new Pose2d(3.17, 4.19, new Rotation2d(Math.toRadians(-1))), // 1
    //         new Pose2d(3.68, 2.96, new Rotation2d(Math.toRadians(58))), // 2
    //         new Pose2d(5.00, 2.79, new Rotation2d(Math.toRadians(118))), // 3
    //         new Pose2d(5.80, 3.84, new Rotation2d(Math.toRadians(180))), // 4
    //         new Pose2d(5.30, 5.07, new Rotation2d(Math.toRadians(-121))), // 5
    //         new Pose2d(3.98, 5.25, new Rotation2d(Math.toRadians(-61))));// 6
    // public static final List<Pose2d> rightBranches = List.of(
    //         new Pose2d(3.17, 3.86, new Rotation2d(Math.toRadians(-1))), // 1
    //         new Pose2d(3.98, 2.80, new Rotation2d(Math.toRadians(58))), // 2
    //         new Pose2d(5.29, 2.96, new Rotation2d(Math.toRadians(118))), // 3
    //         new Pose2d(5.80, 4.19, new Rotation2d(Math.toRadians(180))), // 4
    //         new Pose2d(4.99, 5.25, new Rotation2d(Math.toRadians(-121))), // 5
    //         new Pose2d(3.68, 5.07, new Rotation2d(Math.toRadians(-61))));// 6
    // public static final List<Pose2d> rightBranchL1 = List.of(
    //         new Pose2d(3.06, 3.57, new Rotation2d(Math.toRadians(189))), // front
    //         new Pose2d(5.02, 5.42, new Rotation2d(Math.toRadians(320))), // back left
    //         new Pose2d(5.37, 2.93, new Rotation2d(Math.toRadians(82))), // back right
    //         new Pose2d(3.72, 2.95, new Rotation2d(Math.toRadians(55))), // front right
    //         new Pose2d(3.64, 5.15, new Rotation2d(Math.toRadians(266))), // front left
    //         new Pose2d(5.90, 4.28, new Rotation2d(Math.toRadians(19))));// back
    // public static final List<Pose2d> leftBranchL1 = List.of(
    //         new Pose2d(3.08, 4.08, new Rotation2d(Math.toRadians(155))), // front
    //         new Pose2d(5.25, 5.20, new Rotation2d(Math.toRadians(278))), // back left
    //         new Pose2d(5.00, 2.74, new Rotation2d(Math.toRadians(48))), // back right
    //         new Pose2d(3.76, 2.82, new Rotation2d(Math.toRadians(95))), // front right
    //         new Pose2d(4.02, 5.31, new Rotation2d(Math.toRadians(230))), // front left
    //         new Pose2d(5.87, 3.87, new Rotation2d(Math.toRadians(344))));// back
    // public static final List<Pose2d> coralStations = List.of(
    //         new Pose2d(1.43, 0.72, new Rotation2d(Math.toRadians(55))),
    //         new Pose2d(1.38, 7.31, new Rotation2d(Math.toRadians(-55))));

    // R2OC Red and Blue average Pose
    public static final List<Pose2d> leftBranches =
        List.of(
            new Pose2d(3.182, 4.192, new Rotation2d(Math.toRadians(-1.4))), // 18_LEFT
            new Pose2d(3.682, 2.984, new Rotation2d(Math.toRadians(58.0))), // 17_LEFT
            new Pose2d(4.994, 2.810, new Rotation2d(Math.toRadians(118.1))), // 22_LEFT
            new Pose2d(5.795, 3.849, new Rotation2d(Math.toRadians(178.1))), // 21_LEFT
            new Pose2d(5.287, 5.073, new Rotation2d(Math.toRadians(-121.5))), // 20_LEFT
            new Pose2d(3.991, 5.245, new Rotation2d(Math.toRadians(-62.1)))); // 19_LEFT
    public static final List<Pose2d> rightBranches =
        List.of(
            new Pose2d(3.181, 3.851, new Rotation2d(Math.toRadians(-0.8))), // 18_RIGHT
            new Pose2d(3.985, 2.808, new Rotation2d(Math.toRadians(59.1))), // 17_RIGHT
            new Pose2d(5.288, 2.977, new Rotation2d(Math.toRadians(118.7))), // 22_RIGHT
            new Pose2d(5.796, 4.206, new Rotation2d(Math.toRadians(179.7))), // 21_RIGHT
            new Pose2d(4.989, 5.246, new Rotation2d(Math.toRadians(-121.0))), // 20_RIGHT
            new Pose2d(3.691, 5.073, new Rotation2d(Math.toRadians(-61.4)))); // 19_RIGHT

    // R2OC
    //  public static final List<Pose2d> leftBranches = List.of(
    //          new Pose2d(3.180, 4.135, new Rotation2d(Math.toRadians(-1.6))),   // 18_LEFT
    //          new Pose2d(3.680, 2.980, new Rotation2d(Math.toRadians(57.9))),   // 17_LEFT
    //          new Pose2d(4.995, 2.805, new Rotation2d(Math.toRadians(118.4))),  // 22_LEFT
    //          new Pose2d(5.795, 3.843, new Rotation2d(Math.toRadians(178.1))),  // 21_LEFT
    //          new Pose2d(5.284, 5.071, new Rotation2d(Math.toRadians(-121.2))), // 20_LEFT
    //          new Pose2d(3.989, 5.241, new Rotation2d(Math.toRadians(-62)))); // 19_LEFT
    //  public static final List<Pose2d> rightBranches = List.of(
    //          new Pose2d(3.178, 3.842, new Rotation2d(Math.toRadians(-0.6))),   // 18_RIGHT
    //          new Pose2d(3.985, 2.804, new Rotation2d(Math.toRadians(58.9))),   // 17_RIGHT
    //          new Pose2d(5.286, 2.971, new Rotation2d(Math.toRadians(119))),  // 22_RIGHT
    //          new Pose2d(5.795, 4.204, new Rotation2d(Math.toRadians(179.6))),  // 21_RIGHT
    //          new Pose2d(4.983, 5.244, new Rotation2d(Math.toRadians(-120.3))), // 20_RIGHT
    //          new Pose2d(3.694, 5.072, new Rotation2d(Math.toRadians(-61.6)))); // 19_RIGHT
    public static final List<Pose2d> rightBranchL1 =
        List.of(
            new Pose2d(3.06, 3.57, new Rotation2d(Math.toRadians(189))), // front
            new Pose2d(5.02, 5.42, new Rotation2d(Math.toRadians(320))), // back left
            new Pose2d(5.37, 2.93, new Rotation2d(Math.toRadians(82))), // back right
            new Pose2d(3.72, 2.95, new Rotation2d(Math.toRadians(55))), // front right
            new Pose2d(3.64, 5.15, new Rotation2d(Math.toRadians(266))), // front left
            new Pose2d(5.90, 4.28, new Rotation2d(Math.toRadians(19)))); // back
    public static final List<Pose2d> leftBranchL1 =
        List.of(
            new Pose2d(3.08, 4.08, new Rotation2d(Math.toRadians(155))), // front
            new Pose2d(5.25, 5.20, new Rotation2d(Math.toRadians(278))), // back left
            new Pose2d(5.00, 2.74, new Rotation2d(Math.toRadians(48))), // back right
            new Pose2d(3.76, 2.82, new Rotation2d(Math.toRadians(95))), // front right
            new Pose2d(4.02, 5.31, new Rotation2d(Math.toRadians(230))), // front left
            new Pose2d(5.87, 3.87, new Rotation2d(Math.toRadians(344)))); // back

    // public static final List<Pose2d> coralStations = List.of(
    //         new Pose2d(1.43, 0.72, new Rotation2d(Math.toRadians(55))),
    //         new Pose2d(1.38, 7.31, new Rotation2d(Math.toRadians(-55))));

    // // SHOP
    // public static final List<Pose2d> leftBranches = List.of(
    //        new Pose2d(3.18, 4.23, new Rotation2d(Math.toRadians(-4))), // front (18_LEFT)
    //         new Pose2d(5.24, 5.10, new Rotation2d(Math.toRadians(-120))), // back left (20_LEFT)
    //         new Pose2d(4.96, 2.78, new Rotation2d(Math.toRadians(116))), // back right (22_LEFT)
    //         new Pose2d(3.66, 2.97, new Rotation2d(Math.toRadians(57))), // front right (17_LEFT)
    //         new Pose2d(3.93, 5.21, new Rotation2d(Math.toRadians(-60))), // front left (19_RIGHT)
    //         new Pose2d(5.79, 3.93, new Rotation2d(Math.toRadians(180))));// back (21_LEFT)
    // public static final List<Pose2d> rightBranches = List.of(
    //         new Pose2d(3.18, 3.85, new Rotation2d(Math.toRadians(-2))), // front (18_RIGHT)
    //         new Pose2d(4.93, 5.27, new Rotation2d(Math.toRadians(-120))), // back left (20_RIGHT)
    //         new Pose2d(5.28, 2.95, new Rotation2d(Math.toRadians(118))), // back right (22_RIGHT)
    //         new Pose2d(3.95, 2.80, new Rotation2d(Math.toRadians(58))), // front right (17_RIGHT)
    //         new Pose2d(3.64, 5.04, new Rotation2d(Math.toRadians(-60))), // front left (19_RIGHT)
    //         new Pose2d(5.78, 4.26, new Rotation2d(Math.toRadians(180))));// back (21_RIGHT)
    // public static final List<Pose2d> rightBranchL1 = List.of(
    //         new Pose2d(3.06, 3.57, new Rotation2d(Math.toRadians(189))), // front
    //         new Pose2d(5.02, 5.42, new Rotation2d(Math.toRadians(320))), // back left
    //         new Pose2d(5.37, 2.93, new Rotation2d(Math.toRadians(82))), // back right
    //         new Pose2d(3.72, 2.95, new Rotation2d(Math.toRadians(55))), // front right
    //         new Pose2d(3.64, 5.15, new Rotation2d(Math.toRadians(266))), // front left
    //         new Pose2d(5.90, 4.28, new Rotation2d(Math.toRadians(19))));// back
    // public static final List<Pose2d> leftBranchL1 = List.of(
    //         new Pose2d(3.08, 4.08, new Rotation2d(Math.toRadians(155))), // front
    //         new Pose2d(5.25, 5.20, new Rotation2d(Math.toRadians(278))), // back left
    //         new Pose2d(5.00, 2.74, new Rotation2d(Math.toRadians(48))), // back right
    //         new Pose2d(3.76, 2.82, new Rotation2d(Math.toRadians(95))), // front right
    //         new Pose2d(4.02, 5.31, new Rotation2d(Math.toRadians(230))), // front left
    //         new Pose2d(5.87, 3.87, new Rotation2d(Math.toRadians(344))));// back

    public static final Translation2d reefCenter = new Translation2d(176 * inToM, 158.5 * inToM);
    public static final Translation2d processor = new Translation2d(6, 0);
    public static final double halfwayAcrossFieldY = (317 / 2) * inToM;
    public static final double coralStationLeftHeading = -55;
    public static final double coralStationRightHeading = 55;

    // X value of the translation is irrelevant
    public static final Translation2d netScore = new Translation2d(295 * inToM, 295 * inToM);

    public static final int ATPipelineIndex = 0;

    public static final double xTolerance = Units.inchesToMeters(1);
    public static final double xToleranceHasDistance = Units.inchesToMeters(11.5);
    public static final double yTolerance = Units.inchesToMeters(2);
    public static final double yToleranceHasDistance = Units.inchesToMeters(10);
    public static final double thetaTolerance = 2;
  }
}
