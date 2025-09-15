package frc.robot.io;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Tests for MapleSim swerve drive simulation. */
class MapleSimSwerveTest {
  @BeforeAll
  static void setup() {
    assertTrue(HAL.initialize(500, 0));
  }

  private SelfControlledSwerveDriveSimulation newSim() {
    SwerveDriveSimulation drive =
        new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d());
    return new SelfControlledSwerveDriveSimulation(drive);
  }

  @Test
  void forwardSetpointProducesForwardStates() {
    var sim = newSim();
    sim.runChassisSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0), new Translation2d(), true, true);
    var speeds = DriveConstants.kDriveKinematics.toChassisSpeeds(sim.getSetPointsOptimized());
    assertTrue(speeds.vxMetersPerSecond > 0.5);
    assertEquals(0.0, speeds.vyMetersPerSecond, 1e-6);
    assertEquals(0.0, speeds.omegaRadiansPerSecond, 1e-6);
  }

  @Test
  void maxVelocityMatchesConstant() {
    var sim = newSim();
    double max = sim.maxLinearVelocity().in(edu.wpi.first.units.Units.MetersPerSecond);
    assertEquals(DriveConstants.kMaxSpeedMetersPerSecond, max, 1e-3);
  }
}
