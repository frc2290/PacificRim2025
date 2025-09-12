package frc.utils;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

/** Simple wrapper around PhotonVision simulation objects. */
public class VisionSim {
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final VisionSystemSim visionSim;
    private final SwerveDriveKinematics KINEMATICS = DriveConstants.kDriveKinematics;

    public VisionSim(String cameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(cameraName);

        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(
            VisionConstants.CAMERA_RESOLUTION_WIDTH,
            VisionConstants.CAMERA_RESOLUTION_HEIGHT,
            Rotation2d.fromDegrees(VisionConstants.CAMERA_FOV_DEGREES));
        props.setFPS(VisionConstants.CAMERA_FPS);
        props.setAvgLatencyMs(VisionConstants.CAMERA_AVG_LATENCY_MS);
        props.setLatencyStdDevMs(VisionConstants.CAMERA_LATENCY_STDDEV_MS);
        props.setCalibError(0.25, 0.25);

        cameraSim = new PhotonCameraSim(camera, props);

        AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        visionSim = new VisionSystemSim("main");
        visionSim.addCamera(cameraSim, robotToCamera);
        visionSim.addAprilTags(layout);
    }

    /** Update the simulated camera with the current robot pose. */
    public void updateSim(Pose2d robotPose, double dtSeconds) {
        visionSim.update(robotPose);
    }

    public PhotonCamera getCamera() {
        return camera;
    }
}
