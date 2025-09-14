package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants.DriveConstants;
import frc.utils.PoseEstimatorSubsystem;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class DriveSubsystemSimulationTest {
  @BeforeAll
  public static void setupHAL() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  public void estimatorTracksGroundTruth() {
    RoboRioSim.setVInVoltage(12.0);
    try (DriveSubsystem drive = new DriveSubsystem()) {
      PoseEstimatorSubsystem estimator =
          new PoseEstimatorSubsystem(
              drive::newHeading, drive::getModulePositions, drive::getModuleStates);

      SwerveModuleState[] states =
          DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(1.0, 0.0, 0.0));
      drive.setModuleStates(states);

      for (int i = 0; i < 50; i++) {
        Timer.delay(0.02);
        drive.simulationPeriodic();
        estimator.periodic();
      }

      Pose2d truth = drive.getSimPose();
      Pose2d estimated = estimator.getCurrentPose();
      assertEquals(truth.getX(), estimated.getX(), 0.05);
      assertEquals(truth.getY(), estimated.getY(), 0.05);
      assertEquals(truth.getRotation().getRadians(), estimated.getRotation().getRadians(), 0.05);
    }
  }
}
