package frc.robot.io;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Unit tests for the simulation vision implementation. */
class VisionIOSimTest {
  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void measurementMatchesDrivePose() {
    DriveIOSim driveIO = new DriveIOSim();
    DriveSubsystem drive = new DriveSubsystem(driveIO);
    VisionIOSim vision = new VisionIOSim(drive);
    driveIO.setChassisSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));
    for (int i = 0; i < 5; i++) {
      drive.periodic();
    }
    List<VisionIO.VisionMeasurement> meas = vision.getVisionMeasurements();
    assertEquals(drive.getSimPose(), meas.get(0).pose);
  }
}
