package frc.utils;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.plant.DCMotor;
import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;

/** Unit tests for {@link DifferentialArmSim}. */
public class DifferentialArmSimTest {
  private static final double kMinExtension = 0.0;
  private static final double kMaxExtension = 1.0;
  private static final double kMinTheta = -Math.PI / 2.0;
  private static final double kMaxTheta = Math.PI / 2.0;

  private DifferentialArmSim createSim(double gravity) {
    return new DifferentialArmSim(
        1.0,
        1.0,
        0.2,
        0.5,
        0.0,
        gravity,
        0.0,
        0.0,
        0.0,
        0.0,
        DCMotor.getNEO(1),
        DCMotor.getNEO(1),
        0.1,
        0.1,
        0.0,
        0.0,
        kMinExtension,
        kMaxExtension,
        kMinTheta,
        kMaxTheta,
        0.5,
        0.0);
  }

  @SuppressWarnings("unchecked")
  private <T> T getSimField(DifferentialArmSim sim, String name) throws Exception {
    Field f = edu.wpi.first.wpilibj.simulation.LinearSystemSim.class.getDeclaredField(name);
    f.setAccessible(true);
    return (T) f.get(sim);
  }

  private void setInternalState(DifferentialArmSim sim, int index, double value) throws Exception {
    Matrix<N4, N1> x = getSimField(sim, "m_x");
    Matrix<N4, N1> y = getSimField(sim, "m_y");
    x.set(index, 0, value);
    y.set(index, 0, value);
  }

  @Test
  public void constructorSetsInitialState() {
    DifferentialArmSim sim = createSim(0.0);
    assertEquals(0.5, sim.getExtensionPositionMeters(), 1e-9);
    assertEquals(0.0, sim.getRotationAngleRads(), 1e-9);
  }

  @Test
  public void maxExtensionClampsAndStops() {
    DifferentialArmSim sim = createSim(0.0);
    sim.setState(kMaxExtension - 0.01, 1.0, 0.0, 0.0);
    sim.setInputVoltage(0.0, 0.0);
    sim.update(0.02);
    assertEquals(kMaxExtension, sim.getExtensionPositionMeters(), 1e-9);
    assertEquals(0.0, sim.getExtensionVelocityMetersPerSec(), 1e-9);
  }

  @Test
  public void minAngleClampsAndStops() {
    DifferentialArmSim sim = createSim(9.81);
    sim.setState(0.5, 0.0, kMinTheta + 0.01, -1.0);
    sim.setInputVoltage(0.0, 0.0);
    sim.update(0.02);
    assertEquals(kMinTheta, sim.getRotationAngleRads(), 1e-9);
    assertEquals(0.0, sim.getRotationVelocityRadsPerSec(), 1e-9);
  }

  @Test
  public void setStateClampsWithinLimits() {
    DifferentialArmSim sim = createSim(0.0);
    sim.setState(kMinExtension - 5.0, 0.0, kMinTheta - 5.0, 0.0);
    assertEquals(kMinExtension, sim.getExtensionPositionMeters(), 1e-9);
    assertEquals(kMinTheta, sim.getRotationAngleRads(), 1e-9);
    sim.setState(kMaxExtension + 5.0, 0.0, kMaxTheta + 5.0, 0.0);
    assertEquals(kMaxExtension, sim.getExtensionPositionMeters(), 1e-9);
    assertEquals(kMaxTheta, sim.getRotationAngleRads(), 1e-9);
  }

  @Test
  public void inputVoltageUpdatesInternalVector() throws Exception {
    DifferentialArmSim sim = createSim(0.0);
    sim.setInputVoltage(2.5, -3.0);
    Matrix<N2, N1> u = getSimField(sim, "m_u");
    assertEquals(2.5, u.get(0, 0), 1e-9);
    assertEquals(-3.0, u.get(1, 0), 1e-9);
  }

  @Test
  public void hasHitMinExtensionWorks() throws Exception {
    DifferentialArmSim sim = createSim(0.0);
    double eps = 1e-4;
    setInternalState(sim, 0, kMinExtension - eps);
    assertTrue(sim.hasHitMinExtension());
    setInternalState(sim, 0, kMinExtension);
    assertTrue(sim.hasHitMinExtension());
    setInternalState(sim, 0, kMinExtension + eps);
    assertFalse(sim.hasHitMinExtension());
  }

  @Test
  public void hasHitMaxExtensionWorks() throws Exception {
    DifferentialArmSim sim = createSim(0.0);
    double eps = 1e-4;
    setInternalState(sim, 0, kMaxExtension + eps);
    assertTrue(sim.hasHitMaxExtension());
    setInternalState(sim, 0, kMaxExtension);
    assertTrue(sim.hasHitMaxExtension());
    setInternalState(sim, 0, kMaxExtension - eps);
    assertFalse(sim.hasHitMaxExtension());
  }

  @Test
  public void hasHitMinAngleWorks() throws Exception {
    DifferentialArmSim sim = createSim(0.0);
    double eps = 1e-4;
    setInternalState(sim, 2, kMinTheta - eps);
    assertTrue(sim.hasHitMinAngle());
    setInternalState(sim, 2, kMinTheta);
    assertTrue(sim.hasHitMinAngle());
    setInternalState(sim, 2, kMinTheta + eps);
    assertFalse(sim.hasHitMinAngle());
  }

  @Test
  public void hasHitMaxAngleWorks() throws Exception {
    DifferentialArmSim sim = createSim(0.0);
    double eps = 1e-4;
    setInternalState(sim, 2, kMaxTheta + eps);
    assertTrue(sim.hasHitMaxAngle());
    setInternalState(sim, 2, kMaxTheta);
    assertTrue(sim.hasHitMaxAngle());
    setInternalState(sim, 2, kMaxTheta - eps);
    assertFalse(sim.hasHitMaxAngle());
  }
}
