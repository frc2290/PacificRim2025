package frc.robot.io;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DifferentialArm;
import frc.utils.DifferentialArmSim;
import java.lang.reflect.Field;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Tests validating the differential arm IO simulation mirrors the reference physics. */
public class DifferentialArmIOSimTest {
  private static final double kEps = 1e-4;

  private DifferentialArmSim getInternalSim(DifferentialArmIOSim io) throws Exception {
    Field simField = DifferentialArmIOSim.class.getDeclaredField("sim");
    simField.setAccessible(true);
    return (DifferentialArmSim) simField.get(io);
  }

  private DifferentialArmIOSim io;
  private DifferentialArmSim sim;

  @BeforeEach
  public void setup() throws Exception {
    io = new DifferentialArmIOSim();
    sim = getInternalSim(io);
  }

  @Test
  public void updateReportsCoupledMotorPositionsAndVelocities() {
    // Seed the simulation with a representative coupled state.
    double extensionMeters = 0.2;
    double extensionVelocity = 0.05;
    double rotationRadians = 0.6;
    double rotationVelocity = -0.3;
    sim.setState(extensionMeters, extensionVelocity, rotationRadians, rotationVelocity);

    io.setArmVelocitySetpoints(0.0, 0.0);
    io.update();

    double ext = sim.getExtensionPositionMeters();
    double rot = sim.getRotationAngleRads();
    double extVel = sim.getExtensionVelocityMetersPerSec();
    double rotVel = sim.getRotationVelocityRadsPerSec();

    double rotationCoupling =
        DifferentialArm.kSimDifferentialArmRadiusMeters
            / DifferentialArm.kSimLinearDriveRadiusMeters;

    double expectedLeftMm =
        Units.radiansToRotations(
                ext / DifferentialArm.kSimLinearDriveRadiusMeters + rotationCoupling * rot)
            * DifferentialArm.kEncoderPositionFactor;
    double expectedRightMm =
        Units.radiansToRotations(
                ext / DifferentialArm.kSimLinearDriveRadiusMeters - rotationCoupling * rot)
            * DifferentialArm.kEncoderPositionFactor;
    assertEquals(expectedLeftMm, io.getLeftPosition(), kEps);
    assertEquals(expectedRightMm, io.getRightPosition(), kEps);

    double expectedLeftVelMm =
        Units.radiansPerSecondToRotationsPerMinute(
                extVel / DifferentialArm.kSimLinearDriveRadiusMeters + rotationCoupling * rotVel)
            * DifferentialArm.kEncoderVelocityFactor;
    double expectedRightVelMm =
        Units.radiansPerSecondToRotationsPerMinute(
                extVel / DifferentialArm.kSimLinearDriveRadiusMeters - rotationCoupling * rotVel)
            * DifferentialArm.kEncoderVelocityFactor;
    assertEquals(expectedLeftVelMm, io.getLeftVelocity(), kEps);
    assertEquals(expectedRightVelMm, io.getRightVelocity(), kEps);
  }

  @Test
  public void pureRotationSetpointsMaintainExtension() {
    double initialExtension = sim.getExtensionPositionMeters();

    io.setArmVelocitySetpoints(150.0, -150.0); // Equal and opposite mm/s commands
    for (int i = 0; i < 200; ++i) {
      io.update();
    }

    double finalExtension = sim.getExtensionPositionMeters();
    assertEquals(initialExtension, finalExtension, 1e-4);

    // Confirm the resulting motor velocities remain equal and opposite.
    assertEquals(io.getLeftVelocity(), -io.getRightVelocity(), 1e-6);
  }

  @Test
  public void rotationCommandsFollowReferenceSignConvention() {
    io.setArmVelocitySetpoints(100.0, -100.0);
    for (int i = 0; i < 100; ++i) {
      io.update();
    }
    double positiveAngle = sim.getRotationAngleRads();
    double leftCurrent = io.getLeftCurrentAmps();
    double rightCurrent = io.getRightCurrentAmps();

    assertTrue(positiveAngle > 0.0, "Left-faster command must yield positive rotation");
    assertTrue(leftCurrent > 0.0, "Left motor should source current when driving CCW");
    assertTrue(rightCurrent < 0.0, "Right motor should sink current when driving CCW");

    // Reverse the command and ensure the signs flip.
    io.setArmVelocitySetpoints(-100.0, 100.0);
    for (int i = 0; i < 100; ++i) {
      io.update();
    }
    double negativeAngle = sim.getRotationAngleRads();
    leftCurrent = io.getLeftCurrentAmps();
    rightCurrent = io.getRightCurrentAmps();

    assertTrue(negativeAngle < positiveAngle, "Reversing commands should reduce the angle");
    assertTrue(leftCurrent < 0.0, "Left motor should sink current for CW motion");
    assertTrue(rightCurrent > 0.0, "Right motor should source current for CW motion");
  }
}
