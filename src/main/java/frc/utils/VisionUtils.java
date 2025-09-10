// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionUtils extends SubsystemBase {
  private final PhotonCamera camera;
  private PhotonPipelineResult detections;

  /** Creates a new VisionUtils. */
  public VisionUtils(String cameraName) {
    this(new PhotonCamera(cameraName));
  }

  /**
   * Creates a new VisionUtils with an injected camera for testing.
   *
   * @param camera photon camera instance
   */
  public VisionUtils(PhotonCamera camera) {
    this.camera = camera;
  }

  public PhotonPipelineResult getLatestResult() {
    var results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return new PhotonPipelineResult();
    }
    return results.get(results.size() - 1);
  }

  public PhotonPipelineResult getDetections() {
    return detections;
  }

  public boolean isConnected() {
    return camera.isConnected();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!camera.isConnected()) {
      detections = new PhotonPipelineResult();
    } else {
      detections = getLatestResult();
    }
  }
}
