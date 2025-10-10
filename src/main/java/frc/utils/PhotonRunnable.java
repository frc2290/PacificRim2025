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
package frc.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.utils.PoseUtils.Heading;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** Runnable that gets AprilTag data from PhotonVision. */
public class PhotonRunnable implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final PhotonCamera photonCamera;

  /** Latest pose estimate published from the PhotonVision thread. */
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose =
      new AtomicReference<EstimatedRobotPose>();

  /** Heading supplier used to inject gyro data into the pose estimator. */
  private Supplier<Heading> heading;

  /** Cached pipeline result from the last successful update. */
  private PhotonPipelineResult photonResults;

  private PhotonPipelineResult hasAResult = new PhotonPipelineResult();
  private AprilTagFieldLayout layout;
  private String cameraName;
  private Optional<ConstrainedSolvepnpParams> params =
      Optional.of(new ConstrainedSolvepnpParams(false, 1.0));

  public PhotonRunnable(
      String cam_name, Transform3d cameraToRobot, Supplier<Heading> headingSupplier) {
    cameraName = cam_name;
    heading = headingSupplier;
    this.photonCamera = new PhotonCamera(cameraName);
    ;
    PhotonPoseEstimator photonPoseEstimator = null;
    layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    // PV estimates will always be blue, they'll get flipped by robot thread
    layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
    }
    if (photonCamera != null) {
      photonPoseEstimator =
          new PhotonPoseEstimator(layout, PoseStrategy.CONSTRAINED_SOLVEPNP, cameraToRobot);
    }
    this.photonPoseEstimator = photonPoseEstimator;
  }

  @Override
  public void run() {
    // Get AprilTag data
    if (photonPoseEstimator != null && photonCamera != null) {
      List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();

      for (PhotonPipelineResult result : results) {
        Heading tempHeading = heading.get();
        photonPoseEstimator.addHeadingData(tempHeading.timestamp, tempHeading.rotation);
        Optional<EstimatedRobotPose> photonPose =
            photonPoseEstimator.update(result, Optional.empty(), Optional.empty(), params);
        if (!result.hasTargets()) {
          continue;
        }
        if (photonPose.isPresent()) {
          double tagDist = result.getBestTarget().bestCameraToTarget.getTranslation().getNorm();
          double poseAmbig = result.getBestTarget().getPoseAmbiguity();
          // System.out.println(poseAmbig);
          if (tagDist < 3 && poseAmbig < 0.2) {
            atomicEstimatedRobotPose.set(photonPose.get());
          }
        }
      }
    }
  }

  /**
   * Gets the latest robot pose. Calling this will only return the pose once. If it returns a
   * non-null value, it is a new estimate that hasn't been returned before. This pose will always be
   * for the BLUE alliance. It must be flipped if the current alliance is RED.
   *
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
