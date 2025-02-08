package frc.utils;

//import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT;
import static frc.robot.Constants.VisionConstants.FIELD_LENGTH_METERS;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class PhotonRunnable implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
  private PhotonPipelineResult photonResults;
  private PhotonPipelineResult hasAResult = new PhotonPipelineResult();
  private AprilTagFieldLayout layout;
  private String cameraName;

  public PhotonRunnable(String cam_name) {
    cameraName = cam_name;
    this.photonCamera = new PhotonCamera(cameraName);;
    PhotonPoseEstimator photonPoseEstimator = null;
    //try {
      layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
      // PV estimates will always be blue, they'll get flipped by robot thread
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      if (photonCamera != null) {
        photonPoseEstimator = new PhotonPoseEstimator(
            layout, PoseStrategy.AVERAGE_BEST_TARGETS, APRILTAG_CAMERA_TO_ROBOT.inverse());
      }
    //} catch(IOException e) {
    //  DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
    //  photonPoseEstimator = null;
    //}
    this.photonPoseEstimator = photonPoseEstimator;
  }

  @Override
  public void run() {      
    // Get AprilTag data
    if (photonPoseEstimator != null && photonCamera != null) {
      photonResults = photonCamera.getLatestResult();
      if (photonResults.hasTargets()) {
        //hasAResult = photonResults;
          //&& (photonResults.targets.size() > 1 || photonResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) { 
        photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
          var estimatedPose = estimatedRobotPose.estimatedPose;
          // Make sure the measurement is on the field
          if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= layout.getFieldLength()
              && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= layout.getFieldWidth()) {
            atomicEstimatedRobotPose.set(estimatedRobotPose);
          }
        });
      }
    }  
  }

  /**
   * Gets the latest robot pose. Calling this will only return the pose once. If it returns a non-null value, it is a
   * new estimate that hasn't been returned before.
   * This pose will always be for the BLUE alliance. It must be flipped if the current alliance is RED.
   * @return latest estimated pose
   */
  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }

  public Pose2d grabLatestResult() {
    return layout.getTagPose(hasAResult.getBestTarget().getFiducialId()).get().toPose2d();
  }

  public PhotonPipelineResult grabLatestTag() {
    return photonResults;
  }

  public String getCameraName() {
    return cameraName;
  }
}