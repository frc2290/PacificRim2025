// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import java.util.List;

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

  /** Constants that configure the climb winches and their soft limits. */
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
  }

  /** All elevator setpoints and feedforward values. */
  public static final class Elevator {
    public static final int kLeftElevatorMotorId = 50;
    public static final int kRightElevatorMotorId = 51;

    public static final double kPositionConversion = 0.01683762514; // ((7/34) * (15/31) * (0.0538 *
    // Math.PI)); //
    // Meters Position
    public static final double kVelocityConversion = 0.00028062708; // ((7/34) * (15/31) * (0.0538 *
    // Math.PI)); //
    // Meters Position

    public static final double kP = 0.125;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kKG = 0.0;

    public static final double kS = 0.8842;
    public static final double kV = 5.8979;
    public static final double kA = 0.1156;
    public static final double kG = 0.1987;

    public static final int kCurrentLimitAmps = 50;
    public static final double kPositionToleranceMeters = 0.04;

    public static final double kProfiledKp = 64.0;
    public static final double kProfiledKi = 0.0;
    public static final double kProfiledKd = 1.0;
    public static final double kProfiledMaxVelocityMetersPerSecond = 2.5;
    public static final double kProfiledMaxAccelerationMetersPerSecondSquared = 9.0;
  }

  /** IDs and constants for the manipulator roller. */
  public static final class Manipulator {
    public static final int kManipulatorMotorId = 7;
  }

  /** Extension and rotation constants for the differential arm. */
  public static final class DifferentialArm {
    // Motors
    public static final int kLeftMotorId = 60;
    public static final int kRightMotorId = 61;

    public static final int kArmCurrentLimit = 50;
    public static final double kPositionConversionFactor = 34.2857;
    public static final double kVelocityConversionFactor = 0.5714;
    public static final int kQuadratureMeasurementPeriod = 10;
    public static final int kQuadratureAverageDepth = 2;

    public static final double kVelocityLoopP = 0.00009;
    public static final double kVelocityLoopI = 0.002 * kVelocityLoopP;
    public static final double kVelocityLoopD = 0.0;
    public static final double kVelocityLoopG = 0.0;

    public static final double kExtensionProfiledKp = 30.0;
    public static final double kExtensionProfiledKi = 0.0;
    public static final double kExtensionProfiledKd = 1.5;
    public static final double kExtensionMaxVelocityMillimetersPerSecond = 3000.0;
    public static final double kExtensionMaxAccelerationMillimetersPerSecondSquared = 12000.0;

    public static final double kRotationProfiledKp = 60.0;
    public static final double kRotationProfiledKi = 0.0;
    public static final double kRotationProfiledKd = 4.0;
    public static final double kRotationMaxVelocityDegreesPerSecond = 1400.0;
    public static final double kRotationMaxAccelerationDegreesPerSecondSquared = 5600.0;

    public static final double kExtensionSlewRateMillimetersPerSecond = 700.0;
    public static final double kRotationSlewRateDegreesPerSecond = 120.0;

    public static final double kExtensionPositionToleranceMillimeters = 5.0;
    public static final double kRotationToleranceDegrees = 5.0;

    public static final double kMillimetersPerRotation = 200.0;
    public static final double kDegreesPerRotation = 360.0;

    public static final int kLaserCanId = 5;
    public static final int kLaserRegionX = 8;
    public static final int kLaserRegionY = 8;
    public static final int kLaserRegionWidth = 16;
    public static final int kLaserRegionHeight = 16;
    public static final int kLaserMaxValidDistanceMillimeters = 430;

    // Lookup tables used for interpolating custom scoring positions.
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
  }

  /**
   * Pre-defined manipulator states that keep the arm and elevator synchronized. Each state is
   * referenced by the state machines so that drivers only need to ask for goals by name.
   */
  public static final class ElevatorManipulatorPositions {
    private ElevatorManipulatorPositions() {}

    // ============================ Setpoints ============================
    // Elevator setpoints
    public static final double kElevatorStart = 0.0;
    public static final double kElevatorTransport = 0.25;
    public static final double kElevatorIntake = 0.25;
    public static final double kElevatorL1 = 0.125;
    public static final double kElevatorL2 = 0.5;
    public static final double kElevatorL3 = kElevatorL2 + 0.4;
    public static final double kElevatorL4 = 1.5;
    public static final double kElevatorAlgaeLow = 0.625;
    public static final double kElevatorAlgaeHigh = kElevatorAlgaeLow + 0.4;
    public static final double kElevatorProcessor = 0.125;
    public static final double kElevatorClimb = 0.525;
    public static final double kElevatorBarge = 1.59;

    // Extension setpoints
    public static final double kExtensionStart = 0;
    public static final double kExtensionTransport = 80;
    public static final double kExtensionIntake = 35;
    public static final double kExtensionPrep = 80;
    public static final double kExtensionL1 = 80;
    public static final double kExtensionL2 = 140;
    public static final double kExtensionL3 = kExtensionL2;
    public static final double kExtensionL4 = 215;
    public static final double kExtensionAlgaeIntake = 185;
    public static final double kExtensionProcessor = 80;
    public static final double kExtensionClimb = 80;
    public static final double kExtensionBarge = 140;

    // Rotation setpoints
    public static final double kRotationStart = 0;
    public static final double kRotationTransport = 235;
    public static final double kRotationIntake = 235;
    public static final double kRotationPrep = 235;
    public static final double kRotationL1 = 195;
    public static final double kRotationL2 = 230;
    public static final double kRotationL3 = kRotationL2;
    public static final double kRotationL4 = 80;
    public static final double kRotationAlgaeIntake = 225;
    public static final double kRotationProcessor = 195;
    public static final double kRotationClimb = 80;
    public static final double kRotationBarge = 80;

    // ======================= Travel / Start =======================
    public static final ManipulatorPosition CORAL_TRANSPORT =
        new ManipulatorPosition(kElevatorTransport, kExtensionTransport, kRotationTransport);

    public static final ManipulatorPosition START_POSITION =
        new ManipulatorPosition(kElevatorStart, kExtensionStart, kRotationStart);

    // ========================= Coral Intake =========================
    public static final ManipulatorPosition INTAKE_CORAL =
        new ManipulatorPosition(kElevatorIntake, kExtensionIntake, kRotationIntake);

    // ====================== Coral Scoring (L1–L4) ======================
    public static final ManipulatorPosition L1_PREP =
        new ManipulatorPosition(kElevatorL1, kExtensionPrep, kRotationPrep);
    public static final ManipulatorPosition SCORE_L1 =
        new ManipulatorPosition(kElevatorL1, kExtensionL1, kRotationL1);
    public static final ManipulatorPosition L1_POST_SCORE = L1_PREP;

    public static final ManipulatorPosition L2_PREP =
        new ManipulatorPosition(kElevatorL2, kExtensionPrep, kRotationPrep);
    public static final ManipulatorPosition SCORE_L2 =
        new ManipulatorPosition(kElevatorL2, kExtensionL2, kRotationL2);
    public static final ManipulatorPosition L2_POST_SCORE = L2_PREP;

    public static final ManipulatorPosition L3_PREP =
        new ManipulatorPosition(kElevatorL3, kExtensionPrep, kRotationPrep);
    public static final ManipulatorPosition SCORE_L3 =
        new ManipulatorPosition(kElevatorL3, kExtensionL3, kRotationL3);
    public static final ManipulatorPosition L3_POST_SCORE = L3_PREP;

    public static final ManipulatorPosition L4_PREP =
        new ManipulatorPosition(kElevatorL4, kExtensionPrep, kRotationPrep);
    public static final ManipulatorPosition SCORE_L4 =
        new ManipulatorPosition(kElevatorL4, kExtensionL4, kRotationL4);
    public static final ManipulatorPosition L4_POST_SCORE = L4_PREP;

    // ============================== Algae ==============================
    public static final ManipulatorPosition PREP_ALGAE_LOW =
        new ManipulatorPosition(kElevatorAlgaeLow, kExtensionPrep, kRotationPrep);
    public static final ManipulatorPosition ALGAE_LOW =
        new ManipulatorPosition(kElevatorAlgaeLow, kExtensionAlgaeIntake, kRotationAlgaeIntake);
    public static final ManipulatorPosition PREP_ALGAE_HIGH =
        new ManipulatorPosition(kElevatorAlgaeHigh, kExtensionPrep, kRotationPrep);
    public static final ManipulatorPosition ALGAE_HIGH =
        new ManipulatorPosition(kElevatorAlgaeHigh, kExtensionAlgaeIntake, kRotationAlgaeIntake);

    public static final ManipulatorPosition ALGAE_TRANSPORT = CORAL_TRANSPORT;

    // ========================= Processor / Barge =========================
    public static final ManipulatorPosition SCORE_PROCESSOR =
        new ManipulatorPosition(kElevatorProcessor, kExtensionProcessor, kRotationProcessor);

    public static final ManipulatorPosition PREP_BARGE =
        new ManipulatorPosition(kElevatorBarge, kExtensionPrep, kRotationPrep);
    public static final ManipulatorPosition SCORE_BARGE =
        new ManipulatorPosition(kElevatorBarge, kExtensionBarge, kRotationBarge);
    public static final ManipulatorPosition BARGE_POST_SCORE = PREP_BARGE;

    // =============================== Climb ===============================
    public static final ManipulatorPosition CLIMB =
        new ManipulatorPosition(kElevatorClimb, kExtensionClimb, kRotationClimb);

    // ============================== Misc / Aliases ==============================
    public static final ManipulatorPosition CANCELLED = CORAL_TRANSPORT;

    public record ManipulatorPosition(
        double elevatorMeters, double extensionMillimeters, double rotationDegrees) {}
  }

  /** Drivetrain-specific configuration such as module locations and CAN IDs. */
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.74;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

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
  }

  /** Gear ratios and kinematics values that apply to an individual swerve module. */
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0736;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingVelocityFF = 0.13569;
    public static final double kDrivingP = 0.012343;
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0.0;

    public static final double kTurningP = 4.3865;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;

    public static final int kDrivingCurrentLimit = 50;
    public static final int kTurningCurrentLimit = 20;
    public static final double kNominalVoltage = 12.0;

    public static final double kTurningEncoderFactor = 2 * Math.PI;
  }

  /** Mappings for the driver controller. */
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  /** Motion limits and controller values that are specific to autonomous pathing. */
  public static final class AutoConstants {
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

  /** Specifications that apply to all NEO brushless motors used on the robot. */
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  /** Shared geometry describing the robot's vision sensors and AprilTag layout. */
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
    // new Pose2d(3.17, 4.19, new Rotation2d(Math.toRadians(-1))), // 1
    // new Pose2d(3.68, 2.96, new Rotation2d(Math.toRadians(58))), // 2
    // new Pose2d(5.00, 2.79, new Rotation2d(Math.toRadians(118))), // 3
    // new Pose2d(5.80, 3.84, new Rotation2d(Math.toRadians(180))), // 4
    // new Pose2d(5.30, 5.07, new Rotation2d(Math.toRadians(-121))), // 5
    // new Pose2d(3.98, 5.25, new Rotation2d(Math.toRadians(-61))));// 6
    // public static final List<Pose2d> rightBranches = List.of(
    // new Pose2d(3.17, 3.86, new Rotation2d(Math.toRadians(-1))), // 1
    // new Pose2d(3.98, 2.80, new Rotation2d(Math.toRadians(58))), // 2
    // new Pose2d(5.29, 2.96, new Rotation2d(Math.toRadians(118))), // 3
    // new Pose2d(5.80, 4.19, new Rotation2d(Math.toRadians(180))), // 4
    // new Pose2d(4.99, 5.25, new Rotation2d(Math.toRadians(-121))), // 5
    // new Pose2d(3.68, 5.07, new Rotation2d(Math.toRadians(-61))));// 6
    // public static final List<Pose2d> rightBranchL1 = List.of(
    // new Pose2d(3.06, 3.57, new Rotation2d(Math.toRadians(189))), // front
    // new Pose2d(5.02, 5.42, new Rotation2d(Math.toRadians(320))), // back left
    // new Pose2d(5.37, 2.93, new Rotation2d(Math.toRadians(82))), // back right
    // new Pose2d(3.72, 2.95, new Rotation2d(Math.toRadians(55))), // front right
    // new Pose2d(3.64, 5.15, new Rotation2d(Math.toRadians(266))), // front left
    // new Pose2d(5.90, 4.28, new Rotation2d(Math.toRadians(19))));// back
    // public static final List<Pose2d> leftBranchL1 = List.of(
    // new Pose2d(3.08, 4.08, new Rotation2d(Math.toRadians(155))), // front
    // new Pose2d(5.25, 5.20, new Rotation2d(Math.toRadians(278))), // back left
    // new Pose2d(5.00, 2.74, new Rotation2d(Math.toRadians(48))), // back right
    // new Pose2d(3.76, 2.82, new Rotation2d(Math.toRadians(95))), // front right
    // new Pose2d(4.02, 5.31, new Rotation2d(Math.toRadians(230))), // front left
    // new Pose2d(5.87, 3.87, new Rotation2d(Math.toRadians(344))));// back
    // public static final List<Pose2d> coralStations = List.of(
    // new Pose2d(1.43, 0.72, new Rotation2d(Math.toRadians(55))),
    // new Pose2d(1.38, 7.31, new Rotation2d(Math.toRadians(-55))));

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
    // public static final List<Pose2d> leftBranches = List.of(
    // new Pose2d(3.180, 4.135, new Rotation2d(Math.toRadians(-1.6))), // 18_LEFT
    // new Pose2d(3.680, 2.980, new Rotation2d(Math.toRadians(57.9))), // 17_LEFT
    // new Pose2d(4.995, 2.805, new Rotation2d(Math.toRadians(118.4))), // 22_LEFT
    // new Pose2d(5.795, 3.843, new Rotation2d(Math.toRadians(178.1))), // 21_LEFT
    // new Pose2d(5.284, 5.071, new Rotation2d(Math.toRadians(-121.2))), // 20_LEFT
    // new Pose2d(3.989, 5.241, new Rotation2d(Math.toRadians(-62)))); // 19_LEFT
    // public static final List<Pose2d> rightBranches = List.of(
    // new Pose2d(3.178, 3.842, new Rotation2d(Math.toRadians(-0.6))), // 18_RIGHT
    // new Pose2d(3.985, 2.804, new Rotation2d(Math.toRadians(58.9))), // 17_RIGHT
    // new Pose2d(5.286, 2.971, new Rotation2d(Math.toRadians(119))), // 22_RIGHT
    // new Pose2d(5.795, 4.204, new Rotation2d(Math.toRadians(179.6))), // 21_RIGHT
    // new Pose2d(4.983, 5.244, new Rotation2d(Math.toRadians(-120.3))), // 20_RIGHT
    // new Pose2d(3.694, 5.072, new Rotation2d(Math.toRadians(-61.6)))); // 19_RIGHT
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
    // new Pose2d(1.43, 0.72, new Rotation2d(Math.toRadians(55))),
    // new Pose2d(1.38, 7.31, new Rotation2d(Math.toRadians(-55))));

    // // SHOP
    // public static final List<Pose2d> leftBranches = List.of(
    // new Pose2d(3.18, 4.23, new Rotation2d(Math.toRadians(-4))), // front
    // (18_LEFT)
    // new Pose2d(5.24, 5.10, new Rotation2d(Math.toRadians(-120))), // back left
    // (20_LEFT)
    // new Pose2d(4.96, 2.78, new Rotation2d(Math.toRadians(116))), // back right
    // (22_LEFT)
    // new Pose2d(3.66, 2.97, new Rotation2d(Math.toRadians(57))), // front right
    // (17_LEFT)
    // new Pose2d(3.93, 5.21, new Rotation2d(Math.toRadians(-60))), // front left
    // (19_RIGHT)
    // new Pose2d(5.79, 3.93, new Rotation2d(Math.toRadians(180))));// back
    // (21_LEFT)
    // public static final List<Pose2d> rightBranches = List.of(
    // new Pose2d(3.18, 3.85, new Rotation2d(Math.toRadians(-2))), // front
    // (18_RIGHT)
    // new Pose2d(4.93, 5.27, new Rotation2d(Math.toRadians(-120))), // back left
    // (20_RIGHT)
    // new Pose2d(5.28, 2.95, new Rotation2d(Math.toRadians(118))), // back right
    // (22_RIGHT)
    // new Pose2d(3.95, 2.80, new Rotation2d(Math.toRadians(58))), // front right
    // (17_RIGHT)
    // new Pose2d(3.64, 5.04, new Rotation2d(Math.toRadians(-60))), // front left
    // (19_RIGHT)
    // new Pose2d(5.78, 4.26, new Rotation2d(Math.toRadians(180))));// back
    // (21_RIGHT)
    // public static final List<Pose2d> rightBranchL1 = List.of(
    // new Pose2d(3.06, 3.57, new Rotation2d(Math.toRadians(189))), // front
    // new Pose2d(5.02, 5.42, new Rotation2d(Math.toRadians(320))), // back left
    // new Pose2d(5.37, 2.93, new Rotation2d(Math.toRadians(82))), // back right
    // new Pose2d(3.72, 2.95, new Rotation2d(Math.toRadians(55))), // front right
    // new Pose2d(3.64, 5.15, new Rotation2d(Math.toRadians(266))), // front left
    // new Pose2d(5.90, 4.28, new Rotation2d(Math.toRadians(19))));// back
    // public static final List<Pose2d> leftBranchL1 = List.of(
    // new Pose2d(3.08, 4.08, new Rotation2d(Math.toRadians(155))), // front
    // new Pose2d(5.25, 5.20, new Rotation2d(Math.toRadians(278))), // back left
    // new Pose2d(5.00, 2.74, new Rotation2d(Math.toRadians(48))), // back right
    // new Pose2d(3.76, 2.82, new Rotation2d(Math.toRadians(95))), // front right
    // new Pose2d(4.02, 5.31, new Rotation2d(Math.toRadians(230))), // front left
    // new Pose2d(5.87, 3.87, new Rotation2d(Math.toRadians(344))));// back

    public static final Translation2d reefCenter = new Translation2d(176 * inToM, 158.5 * inToM);
    public static final Translation2d processor = new Translation2d(6, 0);

    /** Pose used when pointing the drivetrain toward the center of the reef. */
    public static final Pose2d REEF_CENTER_AIM_POSE = new Pose2d(reefCenter, new Rotation2d());

    /** Pose used when aiming the drivetrain at the processor. */
    public static final Pose2d PROCESSOR_AIM_POSE = new Pose2d(processor, new Rotation2d());

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
