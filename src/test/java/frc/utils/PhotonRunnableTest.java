package frc.utils;

import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

@ExtendWith(MockitoExtension.class)
class PhotonRunnableTest {
    @Mock private PhotonCamera camera;
    @Mock private PhotonPoseEstimator estimator;

    private PhotonRunnable runnable;

    @BeforeEach
    void setUp() {
        Supplier<PoseUtils.Heading> headingSupplier = () -> new PoseUtils.Heading(0.0, Rotation2d.fromDegrees(0));
        when(camera.getName()).thenReturn("TestCam");
        when(camera.isConnected()).thenReturn(true);
        runnable = new PhotonRunnable(camera, estimator, headingSupplier);
    }

    @Test
    void runWithNoResultsDoesNotUpdatePose() {
        when(camera.getAllUnreadResults()).thenReturn(List.of());

        runnable.run();

        assertNull(runnable.grabLatestEstimatedPose());
        verify(estimator, never()).update(any(), any(), any(), any());
    }

    @Test
    void runStoresPoseWhenValidResult() {
        PhotonTrackedTarget target = new PhotonTrackedTarget();
        target.bestCameraToTarget = new Transform3d(new Translation3d(1, 0, 0), new Rotation3d());
        target.poseAmbiguity = 0.1;
        target.fiducialId = 1;

        PhotonPipelineResult result = new PhotonPipelineResult();
        result.targets = List.of(target);

        EstimatedRobotPose pose = new EstimatedRobotPose(new Pose3d(), 0.0, List.of(target), PoseStrategy.CONSTRAINED_SOLVEPNP);

        when(camera.getAllUnreadResults()).thenReturn(List.of(result));
        when(estimator.update(eq(result), any(), any(), any())).thenReturn(Optional.of(pose));

        runnable.run();

        assertSame(pose, runnable.grabLatestEstimatedPose());
    }

    @Test
    void runSkipsWhenCameraDisconnected() {
        when(camera.isConnected()).thenReturn(false);
        runnable.run();
        verify(estimator, never()).update(any(), any(), any(), any());
        assertNull(runnable.grabLatestEstimatedPose());
    }
}
