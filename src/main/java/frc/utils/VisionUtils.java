// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionUtils extends SubsystemBase {
  PhotonCamera camera;
  PhotonPipelineResult detections;
  
  /** Creates a new VisionUtils. */
  public VisionUtils(String cameraName) {
    camera = new PhotonCamera(cameraName);
  }

  public PhotonPipelineResult getLatestResult() {
    var results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return new PhotonPipelineResult();
    }
    return results.get(results.size() - 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    detections = getLatestResult();
  }
}
