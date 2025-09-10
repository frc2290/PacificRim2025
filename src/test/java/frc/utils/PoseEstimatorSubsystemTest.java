package frc.utils;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import java.util.function.Supplier;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

class PoseEstimatorSubsystemTest {
    private PoseEstimatorSubsystem pose;

    private final Supplier<Rotation2d> rotationSupplier = () -> new Rotation2d();
    private final Supplier<SwerveModulePosition[]> positionSupplier = () -> new SwerveModulePosition[] {
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d())
    };
    private final Supplier<SwerveModuleState[]> stateSupplier = () -> new SwerveModuleState[] {
            new SwerveModuleState(0, new Rotation2d()),
            new SwerveModuleState(0, new Rotation2d()),
            new SwerveModuleState(0, new Rotation2d()),
            new SwerveModuleState(0, new Rotation2d())
    };

    @BeforeEach
    void setUp() {
        pose = new PoseEstimatorSubsystem(rotationSupplier, positionSupplier, stateSupplier);
    }

    @Test
    void atTargetPoseWithinToleranceReturnsTrue() {
        pose.setCurrentPose(new Pose2d(1.0, 1.0, new Rotation2d()));
        pose.setTargetPose(new Pose2d(1.01, 1.02, Rotation2d.fromDegrees(1)));

        assertTrue(pose.atTargetPose());
    }

    @Test
    void atTargetPoseOutsideToleranceReturnsFalse() {
        pose.setCurrentPose(new Pose2d(1.0, 1.0, new Rotation2d()));
        pose.setTargetPose(new Pose2d(1.1, 1.1, Rotation2d.fromDegrees(10)));

        assertFalse(pose.atTargetPose());
    }

    @Test
    void getAlignXAndYReturnExpectedOffsets() {
        pose.setCurrentPose(new Pose2d(1.0, 1.0, new Rotation2d()));
        Translation2d target = new Translation2d(2.0, 3.0);

        assertEquals(1.0, pose.getAlignX(target), 1e-9);
        assertEquals(2.0, pose.getAlignY(target), 1e-9);
    }

    @Test
    void turnToTargetComputesHeading() {
        pose.setCurrentPose(new Pose2d(1.0, 1.0, new Rotation2d()));
        Translation2d target = new Translation2d(2.0, 2.0);

        assertEquals(45.0, pose.turnToTarget(target), 1e-9);
    }

    @Test
    void setAllianceFlipsPoseWhenTagSeen() throws Exception {
        Pose2d original = new Pose2d(1.0, 2.0, new Rotation2d());
        pose.setCurrentPose(original);

        Field sawTagField = PoseEstimatorSubsystem.class.getDeclaredField("sawTag");
        sawTagField.setAccessible(true);
        sawTagField.setBoolean(pose, true);

        pose.setAlliance(Alliance.Red);

        Pose2d expected = original.relativeTo(Constants.VisionConstants.FLIPPING_POSE);
        Pose2d actual = pose.getCurrentPose();

        assertEquals(expected.getX(), actual.getX(), 1e-9);
        assertEquals(expected.getY(), actual.getY(), 1e-9);
        assertEquals(expected.getRotation().getDegrees(), actual.getRotation().getDegrees(), 1e-9);
    }
}
