package frc.utils;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import java.lang.reflect.Field;
import org.ejml.data.Complex_F64;
import org.ejml.simple.SimpleEVD;
import org.junit.jupiter.api.Test;

/** Unit tests for {@link DifferentialArmDynamics}. */
public class DifferentialArmDynamicsTest {
  private static final double kEps = 1e-6;

  private DifferentialArmDynamics createDynamics(double gravity, double extensionInclination) {
    return new DifferentialArmDynamics(
        1.0, // extension mass
        1.0, // rotation mass
        0.2, // rotation inertia
        0.5, // COM offset
        extensionInclination,
        gravity,
        0.0, // extension viscous damping
        0.0, // extension coulomb friction
        0.0, // rotation viscous damping
        0.0, // rotation coulomb friction
        DCMotor.getNEO(1),
        DCMotor.getNEO(1),
        0.1, // linear drive radius
        0.1, // differential arm radius
        0.0, // sensor offset
        0.0 // motor rotor inertia
        );
  }

  @Test
  public void constructorStoresParameters() throws Exception {
    DCMotor right = DCMotor.getNEO(1);
    DCMotor left = DCMotor.getNEO(1);
    double extensionMass = 2.0;
    double rotationMass = 3.0;
    double rotationInertia = 0.7;
    double comOffset = 0.4;
    double extensionInclination = 0.3;
    double gravity = 9.0;
    double extensionViscous = 0.01;
    double extensionCoulomb = 0.02;
    double rotationViscous = 0.03;
    double rotationCoulomb = 0.04;
    double linearDriveRadius = 0.05;
    double diffArmRadius = 0.06;
    double sensorOffset = 0.07;
    double rotorInertia = 0.08;

    DifferentialArmDynamics dynamics =
        new DifferentialArmDynamics(
            extensionMass,
            rotationMass,
            rotationInertia,
            comOffset,
            extensionInclination,
            gravity,
            extensionViscous,
            extensionCoulomb,
            rotationViscous,
            rotationCoulomb,
            right,
            left,
            linearDriveRadius,
            diffArmRadius,
            sensorOffset,
            rotorInertia);

    assertEquals(right, getField(dynamics, "m_rightMotor"));
    assertEquals(left, getField(dynamics, "m_leftMotor"));
    assertEquals(extensionMass, (double) getField(dynamics, "m_extensionMass"), kEps);
    assertEquals(rotationMass, (double) getField(dynamics, "m_rotationMass"), kEps);
    assertEquals(rotationInertia, (double) getField(dynamics, "m_rotationInertia"), kEps);
    assertEquals(comOffset, (double) getField(dynamics, "m_comOffset"), kEps);
    assertEquals(extensionInclination, (double) getField(dynamics, "m_extensionInclination"), kEps);
    assertEquals(gravity, (double) getField(dynamics, "m_gravity"), kEps);
    assertEquals(extensionViscous, (double) getField(dynamics, "m_extensionViscousDamping"), kEps);
    assertEquals(extensionCoulomb, (double) getField(dynamics, "m_extensionCoulombFriction"), kEps);
    assertEquals(rotationViscous, (double) getField(dynamics, "m_rotationViscousDamping"), kEps);
    assertEquals(rotationCoulomb, (double) getField(dynamics, "m_rotationCoulombFriction"), kEps);
    assertEquals(linearDriveRadius, (double) getField(dynamics, "m_linearDriveRadius"), kEps);
    assertEquals(diffArmRadius, (double) getField(dynamics, "m_differentialArmRadius"), kEps);
    assertEquals(sensorOffset, (double) getField(dynamics, "m_sensorOffset"), kEps);
    assertEquals(rotorInertia, (double) getField(dynamics, "m_motorRotorInertia"), kEps);
  }

  private Object getField(Object obj, String name) throws Exception {
    Field f = obj.getClass().getDeclaredField(name);
    f.setAccessible(true);
    return f.get(obj);
  }

  private Matrix<N4, N1> state(double angle) {
    return VecBuilder.fill(0.0, 0.0, angle, 0.0);
  }

  @Test
  public void zeroInputsZeroGravityNoMotion() {
    DifferentialArmDynamics dyn = createDynamics(0.0, 0.0);
    Matrix<N4, N1> x = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> u = VecBuilder.fill(0.0, 0.0);
    Matrix<N4, N1> dx = dyn.dynamics(x, u);
    assertEquals(0.0, dx.get(1, 0), 1e-9);
    assertEquals(0.0, dx.get(3, 0), 1e-9);
  }

  @Test
  public void gravityCausesArmToFall() {
    DifferentialArmDynamics dyn = createDynamics(9.81, 0.0);
    Matrix<N4, N1> x = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> u = VecBuilder.fill(0.0, 0.0);
    Matrix<N4, N1> dx = dyn.dynamics(x, u);
    assertTrue(dx.get(3, 0) < 0.0);
  }

  @Test
  public void gravityCausesExtensionSlide() {
    // Negative inclination corresponds to axis sloping downward in positive r direction
    DifferentialArmDynamics dyn = createDynamics(9.81, -0.2);
    Matrix<N4, N1> x = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> u = VecBuilder.fill(0.0, 0.0);
    Matrix<N4, N1> dx = dyn.dynamics(x, u);
    assertTrue(dx.get(1, 0) > 0.0);
  }

  @Test
  public void opposingVoltagesRotateArm() {
    DifferentialArmDynamics dyn = createDynamics(0.0, 0.0);
    Matrix<N4, N1> x = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> u = VecBuilder.fill(-5.0, 5.0);
    Matrix<N4, N1> dx = dyn.dynamics(x, u);
    assertTrue(dx.get(3, 0) > 0.0);
    assertEquals(0.0, dx.get(1, 0), 1e-3);
  }

  @Test
  public void equalVoltagesExtendArm() {
    DifferentialArmDynamics dyn = createDynamics(0.0, 0.0);
    Matrix<N4, N1> x = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> u = VecBuilder.fill(5.0, 5.0);
    Matrix<N4, N1> dx = dyn.dynamics(x, u);
    assertTrue(dx.get(1, 0) > 0.0);
    assertEquals(0.0, dx.get(3, 0), 1e-3);
  }

  @Test
  public void feedforwardHoldsHorizontal() {
    DifferentialArmDynamics dyn = createDynamics(9.81, 0.0);
    Matrix<N4, N1> x = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> ff = dyn.calculateFeedforward(x);
    assertNotEquals(0.0, ff.get(0, 0), kEps);
    assertNotEquals(0.0, ff.get(1, 0), kEps);
    Matrix<N4, N1> dx = dyn.dynamics(x, ff);
    assertEquals(0.0, dx.get(1, 0), 1e-6);
    assertEquals(0.0, dx.get(3, 0), 1e-6);
  }

  @Test
  public void feedforwardVerticalHasNoRotationComponent() {
    DifferentialArmDynamics dyn = createDynamics(9.81, 0.0);
    Matrix<N4, N1> x = VecBuilder.fill(0.0, 0.0, -Math.PI / 2.0, 0.0);
    Matrix<N2, N1> ff = dyn.calculateFeedforward(x);
    assertEquals(ff.get(0, 0), ff.get(1, 0), 1e-6);
  }

  @Test
  public void gravityEffectsVaryWithAngle() {
    DifferentialArmDynamics dyn = createDynamics(9.81, 0.0);
    Matrix<N4, N1> horizontal = state(0.0);
    Matrix<N4, N1> vertical = state(Math.PI / 2.0);
    Matrix<N2, N1> horizV = dyn.calculateFeedforward(horizontal);
    Matrix<N2, N1> vertV = dyn.calculateFeedforward(vertical);
    assertTrue(Math.abs(horizV.get(0, 0)) > Math.abs(vertV.get(0, 0)));
    assertTrue(Math.abs(horizV.get(1, 0)) > Math.abs(vertV.get(1, 0)));
  }

  @Test
  public void feedforwardMatchesAnalyticalTorque() {
    DifferentialArmDynamics dyn = createDynamics(9.81, 0.0);
    double theta = Math.PI / 4.0;
    Matrix<N4, N1> x = state(theta);
    Matrix<N2, N1> volts = dyn.calculateFeedforward(x);
    double g2 = 1.0 * 9.81 * 0.5 * Math.cos(theta);
    double tauR = (-g2 * 0.1 / 0.1) / 2.0;
    double tauL = (g2 * 0.1 / 0.1) / 2.0;
    DCMotor motor = DCMotor.getNEO(1);
    double expectedR = motor.getVoltage(tauR, 0.0);
    double expectedL = motor.getVoltage(tauL, 0.0);
    assertEquals(expectedR, volts.get(0, 0), 1e-9);
    assertEquals(expectedL, volts.get(1, 0), 1e-9);
  }

  @Test
  public void linearizedSystemHasCorrectDimensions() {
    DifferentialArmDynamics dyn = createDynamics(9.81, 0.0);
    LinearSystem<N4, N2, N4> sys = dyn.createLinearizedSystem(0.5, -Math.PI / 2.0);
    Matrix<N4, N4> A = sys.getA();
    Matrix<N4, N2> B = sys.getB();
    assertEquals(4, A.getNumRows());
    assertEquals(4, A.getNumCols());
    assertEquals(4, B.getNumRows());
    assertEquals(2, B.getNumCols());
  }

  @Test
  public void equilibriumLinearizationIsStable() {
    DifferentialArmDynamics dyn = createDynamics(9.81, 0.0);
    LinearSystem<N4, N2, N4> sys = dyn.createLinearizedSystem(0.5, -Math.PI / 2.0);
    SimpleEVD<?> evd = sys.getA().getStorage().eig();
    for (int i = 0; i < evd.getNumberOfEigenvalues(); i++) {
      Complex_F64 eig = evd.getEigenvalue(i);
      assertTrue(eig.getReal() <= kEps);
    }
  }

  @Test
  public void stallCurrentMatchesMotorModel() {
    DifferentialArmDynamics dyn = createDynamics(0.0, 0.0);
    DCMotor motor = DCMotor.getNEO(1);
    double volts = 2.0;
    Matrix<N4, N1> x = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> u = VecBuilder.fill(volts, volts);
    double expected = volts / motor.rOhms;
    double iR = dyn.getRightMotorCurrentAmps(x, u);
    double iL = dyn.getLeftMotorCurrentAmps(x, u);
    assertEquals(expected, iR, 1e-6);
    assertEquals(expected, iL, 1e-6);
    assertEquals(Math.abs(iR) + Math.abs(iL), dyn.getTotalCurrentAbsAmps(x, u), 1e-6);
  }

  @Test
  public void freeSpeedCurrentNearFreeCurrent() {
    DifferentialArmDynamics dyn = createDynamics(0.0, 0.0);
    DCMotor motor = DCMotor.getNEO(1);
    double v = motor.nominalVoltageVolts;
    double omega = motor.freeSpeedRadPerSec;
    double rDot = omega * 0.1; // linearDriveRadius
    Matrix<N4, N1> x = VecBuilder.fill(0.0, rDot, 0.0, 0.0);
    Matrix<N2, N1> u = VecBuilder.fill(v, v);
    double iR = dyn.getRightMotorCurrentAmps(x, u);
    double iL = dyn.getLeftMotorCurrentAmps(x, u);
    assertEquals(motor.freeCurrentAmps, iR, 0.5);
    assertEquals(motor.freeCurrentAmps, iL, 0.5);
    assertEquals(Math.abs(iR) + Math.abs(iL), dyn.getTotalCurrentAbsAmps(x, u), 0.5);
  }
}
