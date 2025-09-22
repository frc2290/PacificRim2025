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
