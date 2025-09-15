package frc.robot.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;
import org.photonvision.PhotonCamera;

/** Simulation vision implementation using MapleSim world pose. */
public class VisionIOSim implements VisionIO {
  private final DriveSubsystem drive;

  public VisionIOSim(DriveSubsystem drive) {
    this.drive = drive;
    PhotonCamera.setVersionCheckEnabled(false);
  }

  @Override
  public List<VisionMeasurement> getVisionMeasurements() {
    Pose2d pose = drive.getSimPose();
    return List.of(new VisionMeasurement(pose, Timer.getFPGATimestamp()));
  }
}
