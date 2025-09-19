// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Thin wrapper around a PhotonCamera that refreshes its latest result each loop. */
public class VisionUtils extends SubsystemBase {
  /** Underlying PhotonVision camera. */
  PhotonCamera camera;
  /** Cached copy of the most recent pipeline result. */
  PhotonPipelineResult detections;
  
  /** Creates a new VisionUtils. */
  public VisionUtils(String cameraName) {
    camera = new PhotonCamera(cameraName);
  }

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    detections = getLatestResult();
  }
}
