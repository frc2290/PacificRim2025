// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

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
