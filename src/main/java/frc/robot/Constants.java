// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.math.util.Units.degreesToRadians;

import java.util.List;

import com.pathplanner.lib.path.PathConstraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final boolean debugMode = true;

    public static final class Elevator {
        public static final int kLeftElevatorMotorId = 50;
        public static final int kRightElevatorMotorId = 51;

        public static final double kPositionConversion = 0.01683762514; // ((7/34) * (15/31) * (0.0538 * Math.PI)); //
                                                                        // Meters Position
        public static final double kVelocityConversion = 0.00028062708; // ((7/34) * (15/31) * (0.0538 * Math.PI)); //
                                                                        // Meters Position

        public static final double kP = 0.125;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kKG = 0.0;

        public static final double transportSetpoint = 0.375;
        public static final double intakeSetpoint = 0.1;
    }

    public static final class Manipulator {
        public static final int kManipulatorMotorId = 7;
    }

    public static final class DifferentialArm {
        // Motors
        public static final int kLeftMotorId = 60;
        public static final int kRightMotorId = 61;
        // configuration
        public static final double voltageComp = 0;
        public static final int currentStallLim = 0;
        public static final int currentFreeLim = 0;

        public static final double v_kp = 0.015;
        public static final double v_ki = 0.00005;
        public static final double v_kd = 0;
        public static final double v_KG = 0;

        public static final double transportExtensionSetpoint = 20;
        public static final double transportRotationSetpoint = -125;
        public static final double intakeExtensionSetpoint = 0;
        public static final double intakeRotationSetpoint = 8;

        // pid tune values Internal loop
        // public static final double v_kp = 0.000006;
        // public static final double v_ki = 0.000000005;
        // public static final double v_kd = 0.000001;
        // public static final double v_KG = 0;

        // pid tune values external extension
        public static final double e_kp = 0;
        public static final double e_ki = 0;
        public static final double e_kd = 0;

        // pid tune values external rotation
        public static final double r_kp = 0;
        public static final double r_ki = 0;
        public static final double r_kd = 0;
    }

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
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
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
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        public static final PathConstraints kPathConstraints = new PathConstraints(kMaxSpeedMetersPerSecond, 
                                                                                    kMaxAccelerationMetersPerSecondSquared,
                                                                                    kMaxAngularSpeedRadiansPerSecond, 
                                                                                    kMaxAngularSpeedRadiansPerSecondSquared);

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 6784;
    }

    public static final class VisionConstants {
        public static final double CAMERA_HEIGHT_METERS = 0.9144;
        public static final double CAMERA_PITCH_RADIANS = degreesToRadians(45);
        public static final double TARGET_HEIGHT_METERS = 1.4351;
        /**
         * Physical location of the apriltag camera on the robot, relative to the center
         * of the robot.
         */
        public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(
                new Translation3d(0.142875, -0.36, CAMERA_HEIGHT_METERS),
                new Rotation3d(0.0, CAMERA_PITCH_RADIANS, degreesToRadians(20)));

        public static final Transform3d APRILTAG_CAMERA2_TO_ROBOT = new Transform3d(
            new Translation3d((0.142875-0.152), -0.36, CAMERA_HEIGHT_METERS),
            new Rotation3d(0.0, 0, degreesToRadians(160)));

        public static final double FIELD_LENGTH_METERS = 16.54175;
        public static final double FIELD_WIDTH_METERS = 8.0137;

        // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose
        // to the opposite alliance
        public static final Pose2d FLIPPING_POSE = new Pose2d(
                new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
                new Rotation2d(Math.PI));

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
        public static final List<Pose2d> leftBranches = List.of(
                new Pose2d(3.18, 4.48, new Rotation2d(Math.toRadians(-2.75))), // front
                new Pose2d(5.24, 5.10, new Rotation2d(Math.toRadians(-120))), // back left
                new Pose2d(4.73, 2.66, new Rotation2d(Math.toRadians(113))), // back right
                new Pose2d(3.44, 3.12, new Rotation2d(Math.toRadians(55))), // front right
                new Pose2d(3.93, 5.21, new Rotation2d(Math.toRadians(-60))), // front left
                new Pose2d(5.79, 3.93, new Rotation2d(Math.toRadians(180))));// back
        public static final List<Pose2d> rightBranches = List.of(
                new Pose2d(3.15, 4.11, new Rotation2d(Math.toRadians(-2.75))), // front
                new Pose2d(4.93, 5.27, new Rotation2d(Math.toRadians(-120))), // back left
                new Pose2d(5.04, 2.82, new Rotation2d(Math.toRadians(114))), // back right
                new Pose2d(4.05, 2.79, new Rotation2d(Math.toRadians(60))), // front right
                new Pose2d(3.64, 5.04, new Rotation2d(Math.toRadians(-60))), // front left
                new Pose2d(5.78, 4.26, new Rotation2d(Math.toRadians(180))));// back
        public static final List<Pose2d> rightBranchL1 = List.of(
                new Pose2d(3.06, 3.57, new Rotation2d(Math.toRadians(189))), // front
                new Pose2d(5.02, 5.42, new Rotation2d(Math.toRadians(320))), // back left
                new Pose2d(5.37, 2.93, new Rotation2d(Math.toRadians(82))), // back right
                new Pose2d(3.72, 2.95, new Rotation2d(Math.toRadians(55))), // front right
                new Pose2d(3.64, 5.15, new Rotation2d(Math.toRadians(266))), // front left
                new Pose2d(5.90, 4.28, new Rotation2d(Math.toRadians(19))));// back
        public static final List<Pose2d> leftBranchL1 = List.of(
                new Pose2d(3.08, 4.08, new Rotation2d(Math.toRadians(155))), // front
                new Pose2d(5.25, 5.20, new Rotation2d(Math.toRadians(278))), // back left
                new Pose2d(5.00, 2.74, new Rotation2d(Math.toRadians(48))), // back right
                new Pose2d(3.76, 2.82, new Rotation2d(Math.toRadians(95))), // front right
                new Pose2d(4.02, 5.31, new Rotation2d(Math.toRadians(230))), // front left
                new Pose2d(5.87, 3.87, new Rotation2d(Math.toRadians(344))));// back

        public static final Translation2d reefCenter = new Translation2d(176 * inToM, 158.5 * inToM);
        public static final double halfwayAcrossFieldY = (317 / 2) * inToM;
        public static final double coralStationLeftHeading = -50;
        public static final double coralStationRightHeading = 50;

        // X value of the translation is irrelevant
        public static final Translation2d netScore = new Translation2d(295 * inToM, 295 * inToM);

        public static final int ATPipelineIndex = 0;
    }
}