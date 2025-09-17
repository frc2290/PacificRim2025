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
import java.util.function.Consumer;
import org.ejml.data.Complex_F64;
import org.ejml.simple.SimpleEVD;
import org.junit.jupiter.api.Test;

/** Unit tests for {@link DifferentialArmDynamics}. */
public class DifferentialArmDynamicsTest {
  private static final double kEps = 1e-6;

  private static class Params {
    double extensionMass = 1.0;
    double rotationMass = 1.0;
    double rotationInertia = 0.2;
    double comOffset = 0.5;
    double extensionInclination = 0.0;
    double gravity = 0.0;
    double extensionViscousDamping = 0.0;
    double extensionCoulombFriction = 0.0;
    double rotationViscousDamping = 0.0;
    double rotationCoulombFriction = 0.0;
    DCMotor rightMotor = DCMotor.getNEO(1);
    DCMotor leftMotor = DCMotor.getNEO(1);
    double linearDriveRadius = 0.1;
    double differentialArmRadius = 0.1;
    double sensorOffset = 0.0;
    double motorRotorInertia = 0.0;
  }

  private DifferentialArmDynamics createDynamics(Consumer<Params> configurator) {
    Params params = new Params();
    if (configurator != null) {
      configurator.accept(params);
    }
    return new DifferentialArmDynamics(
        params.extensionMass,
        params.rotationMass,
        params.rotationInertia,
        params.comOffset,
        params.extensionInclination,
        params.gravity,
        params.extensionViscousDamping,
        params.extensionCoulombFriction,
        params.rotationViscousDamping,
        params.rotationCoulombFriction,
        params.rightMotor,
        params.leftMotor,
        params.linearDriveRadius,
        params.differentialArmRadius,
        params.sensorOffset,
        params.motorRotorInertia);
  }

  private DifferentialArmDynamics createDynamics(double gravity, double extensionInclination) {
    return createDynamics(
        params -> {
          params.gravity = gravity;
          params.extensionInclination = extensionInclination;
        });
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

  @Test
  public void higherExtensionMassReducesAcceleration() {
    Matrix<N4, N1> state = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> input = VecBuilder.fill(4.0, 4.0);
    DifferentialArmDynamics light =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.extensionMass = 1.0;
            });
    DifferentialArmDynamics heavy =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.extensionMass = 5.0;
            });
    double lightAccel = light.dynamics(state, input).get(1, 0);
    double heavyAccel = heavy.dynamics(state, input).get(1, 0);
    assertTrue(lightAccel > heavyAccel, "Heavier extension mass should reduce acceleration");
  }

  @Test
  public void largerRotationMassReducesAngularAcceleration() {
    Matrix<N4, N1> state = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> input = VecBuilder.fill(-4.0, 4.0);
    DifferentialArmDynamics light =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.rotationMass = 1.0;
            });
    DifferentialArmDynamics heavy =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.rotationMass = 5.0;
            });
    double lightAccel = light.dynamics(state, input).get(3, 0);
    double heavyAccel = heavy.dynamics(state, input).get(3, 0);
    assertTrue(lightAccel > heavyAccel, "Heavier rotation mass should reduce angular acceleration");
  }

  @Test
  public void largerRotationInertiaReducesAngularAcceleration() {
    Matrix<N4, N1> state = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> input = VecBuilder.fill(-4.0, 4.0);
    DifferentialArmDynamics light =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.rotationInertia = 0.1;
            });
    DifferentialArmDynamics heavy =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.rotationInertia = 1.0;
            });
    double lightAccel = light.dynamics(state, input).get(3, 0);
    double heavyAccel = heavy.dynamics(state, input).get(3, 0);
    assertTrue(lightAccel > heavyAccel, "Higher inertia should reduce angular acceleration");
  }

  @Test
  public void zeroComOffsetEliminatesGravityTorque() {
    Matrix<N4, N1> state = VecBuilder.fill(0.0, 0.0, 0.2, 0.0);
    Matrix<N2, N1> input = VecBuilder.fill(0.0, 0.0);
    DifferentialArmDynamics withOffset =
        createDynamics(
            params -> {
              params.gravity = 9.81;
              params.comOffset = 0.5;
            });
    DifferentialArmDynamics noOffset =
        createDynamics(
            params -> {
              params.gravity = 9.81;
              params.comOffset = 0.0;
            });
    double accelWithOffset = withOffset.dynamics(state, input).get(3, 0);
    double accelNoOffset = noOffset.dynamics(state, input).get(3, 0);
    assertTrue(accelWithOffset < 0.0, "Positive COM offset should create restoring torque");
    assertEquals(0.0, accelNoOffset, 1e-9);
  }

  @Test
  public void extensionInclinationChangesGravityProjection() {
    Matrix<N4, N1> state = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> input = VecBuilder.fill(0.0, 0.0);
    DifferentialArmDynamics uphill =
        createDynamics(
            params -> {
              params.gravity = 9.81;
              params.extensionInclination = 0.2;
            });
    DifferentialArmDynamics downhill =
        createDynamics(
            params -> {
              params.gravity = 9.81;
              params.extensionInclination = -0.2;
            });
    double uphillAccel = uphill.dynamics(state, input).get(1, 0);
    double downhillAccel = downhill.dynamics(state, input).get(1, 0);
    assertTrue(uphillAccel < 0.0);
    assertTrue(downhillAccel > 0.0);
  }

  @Test
  public void extensionViscousDampingOpposesVelocity() {
    Matrix<N4, N1> state = VecBuilder.fill(0.0, 1.0, 0.0, 0.0);
    Matrix<N2, N1> input = VecBuilder.fill(0.0, 0.0);
    DifferentialArmDynamics noDamping = createDynamics(params -> params.gravity = 0.0);
    DifferentialArmDynamics withDamping =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.extensionViscousDamping = 5.0;
            });
    double baseAccel = noDamping.dynamics(state, input).get(1, 0);
    double dampedAccel = withDamping.dynamics(state, input).get(1, 0);
    assertTrue(baseAccel < 0.0);
    assertTrue(dampedAccel < baseAccel);
  }

  @Test
  public void extensionCoulombFrictionOpposesVelocity() {
    Matrix<N4, N1> state = VecBuilder.fill(0.0, 0.05, 0.0, 0.0);
    Matrix<N2, N1> input = VecBuilder.fill(0.0, 0.0);
    DifferentialArmDynamics noFriction = createDynamics(params -> params.gravity = 0.0);
    DifferentialArmDynamics withFriction =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.extensionCoulombFriction = 5.0;
            });
    double baseAccel = noFriction.dynamics(state, input).get(1, 0);
    double frictionAccel = withFriction.dynamics(state, input).get(1, 0);
    assertTrue(baseAccel < 0.0);
    assertTrue(frictionAccel < baseAccel);
  }

  @Test
  public void rotationViscousDampingOpposesAngularVelocity() {
    Matrix<N4, N1> state = VecBuilder.fill(0.0, 0.0, 0.0, 1.0);
    Matrix<N2, N1> input = VecBuilder.fill(0.0, 0.0);
    DifferentialArmDynamics noDamping =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.comOffset = 0.0;
            });
    DifferentialArmDynamics withDamping =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.comOffset = 0.0;
              params.rotationViscousDamping = 5.0;
            });
    double baseAccel = noDamping.dynamics(state, input).get(3, 0);
    double dampedAccel = withDamping.dynamics(state, input).get(3, 0);
    assertTrue(baseAccel < 0.0);
    assertTrue(dampedAccel < baseAccel);
  }

  @Test
  public void rotationCoulombFrictionOpposesAngularVelocity() {
    Matrix<N4, N1> state = VecBuilder.fill(0.0, 0.0, 0.0, 0.1);
    Matrix<N2, N1> input = VecBuilder.fill(0.0, 0.0);
    DifferentialArmDynamics noFriction =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.comOffset = 0.0;
            });
    DifferentialArmDynamics withFriction =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.comOffset = 0.0;
              params.rotationCoulombFriction = 3.0;
            });
    double baseAccel = noFriction.dynamics(state, input).get(3, 0);
    double frictionAccel = withFriction.dynamics(state, input).get(3, 0);
    assertTrue(baseAccel < 0.0);
    assertTrue(frictionAccel < baseAccel);
  }

  @Test
  public void sensorOffsetShiftsAbsoluteAngleOnly() {
    Matrix<N4, N1> baseState = VecBuilder.fill(0.0, 0.0, 0.2, 0.0);
    Matrix<N2, N1> input = VecBuilder.fill(0.0, 0.0);
    DifferentialArmDynamics zeroOffset =
        createDynamics(
            params -> {
              params.gravity = 9.81;
              params.sensorOffset = 0.0;
            });
    DifferentialArmDynamics shiftedOffset =
        createDynamics(
            params -> {
              params.gravity = 9.81;
              params.sensorOffset = 0.3;
            });
    Matrix<N4, N1> shiftedState = VecBuilder.fill(0.0, 0.0, -0.1, 0.0);
    Matrix<N4, N1> dxZero = zeroOffset.dynamics(baseState, input);
    Matrix<N4, N1> dxShifted = shiftedOffset.dynamics(shiftedState, input);
    assertEquals(dxZero.get(1, 0), dxShifted.get(1, 0), 1e-9);
    assertEquals(dxZero.get(3, 0), dxShifted.get(3, 0), 1e-9);
  }

  @Test
  public void smallerLinearDriveRadiusIncreasesExtensionAcceleration() {
    Matrix<N4, N1> state = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> input = VecBuilder.fill(4.0, 4.0);
    DifferentialArmDynamics largeRadius =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.linearDriveRadius = 0.2;
            });
    DifferentialArmDynamics smallRadius =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.linearDriveRadius = 0.05;
            });
    double accelLarge = largeRadius.dynamics(state, input).get(1, 0);
    double accelSmall = smallRadius.dynamics(state, input).get(1, 0);
    assertTrue(accelSmall > accelLarge);
  }

  @Test
  public void largerDifferentialRadiusIncreasesRotationAcceleration() {
    Matrix<N4, N1> state = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> input = VecBuilder.fill(-4.0, 4.0);
    DifferentialArmDynamics smallRadius =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.differentialArmRadius = 0.05;
            });
    DifferentialArmDynamics largeRadius =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.differentialArmRadius = 0.2;
            });
    double accelSmall = smallRadius.dynamics(state, input).get(3, 0);
    double accelLarge = largeRadius.dynamics(state, input).get(3, 0);
    assertTrue(accelLarge > accelSmall);
  }

  @Test
  public void motorRotorInertiaAddsApparentMass() {
    Matrix<N4, N1> state = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> input = VecBuilder.fill(4.0, 4.0);
    DifferentialArmDynamics noRotor =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.motorRotorInertia = 0.0;
            });
    DifferentialArmDynamics heavyRotor =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.motorRotorInertia = 0.5;
            });
    double accelNoRotor = noRotor.dynamics(state, input).get(1, 0);
    double accelHeavyRotor = heavyRotor.dynamics(state, input).get(1, 0);
    assertTrue(accelNoRotor > accelHeavyRotor);
  }

  @Test
  public void strongerRightMotorIncreasesExtensionAcceleration() {
    Matrix<N4, N1> state = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> input = VecBuilder.fill(6.0, 6.0);
    DifferentialArmDynamics baseline = createDynamics(params -> params.gravity = 0.0);
    DifferentialArmDynamics strongerRight =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.rightMotor = DCMotor.getNEO(2);
            });
    double baseAccel = baseline.dynamics(state, input).get(1, 0);
    double boostedAccel = strongerRight.dynamics(state, input).get(1, 0);
    assertTrue(boostedAccel > baseAccel);
  }

  @Test
  public void strongerLeftMotorIncreasesRotationAcceleration() {
    Matrix<N4, N1> state = VecBuilder.fill(0.0, 0.0, 0.0, 0.0);
    Matrix<N2, N1> input = VecBuilder.fill(-6.0, 6.0);
    DifferentialArmDynamics baseline = createDynamics(params -> params.gravity = 0.0);
    DifferentialArmDynamics strongerLeft =
        createDynamics(
            params -> {
              params.gravity = 0.0;
              params.leftMotor = DCMotor.getNEO(2);
            });
    double baseAccel = baseline.dynamics(state, input).get(3, 0);
    double boostedAccel = strongerLeft.dynamics(state, input).get(3, 0);
    assertTrue(boostedAccel > baseAccel);
  }

  @Test
  public void constructorRejectsNonPositiveExtensionMass() {
    assertThrows(
        IllegalArgumentException.class, () -> createDynamics(params -> params.extensionMass = 0.0));
  }

  @Test
  public void constructorRejectsNonPositiveRotationMass() {
    assertThrows(
        IllegalArgumentException.class, () -> createDynamics(params -> params.rotationMass = 0.0));
  }

  @Test
  public void constructorRejectsNegativeRotationInertia() {
    assertThrows(
        IllegalArgumentException.class,
        () -> createDynamics(params -> params.rotationInertia = -0.1));
  }

  @Test
  public void constructorRejectsNegativeComOffset() {
    assertThrows(
        IllegalArgumentException.class, () -> createDynamics(params -> params.comOffset = -0.01));
  }

  @Test
  public void constructorRejectsNegativeExtensionViscousDamping() {
    assertThrows(
        IllegalArgumentException.class,
        () -> createDynamics(params -> params.extensionViscousDamping = -0.1));
  }

  @Test
  public void constructorRejectsNegativeExtensionCoulombFriction() {
    assertThrows(
        IllegalArgumentException.class,
        () -> createDynamics(params -> params.extensionCoulombFriction = -0.1));
  }

  @Test
  public void constructorRejectsNegativeRotationViscousDamping() {
    assertThrows(
        IllegalArgumentException.class,
        () -> createDynamics(params -> params.rotationViscousDamping = -0.1));
  }

  @Test
  public void constructorRejectsNegativeRotationCoulombFriction() {
    assertThrows(
        IllegalArgumentException.class,
        () -> createDynamics(params -> params.rotationCoulombFriction = -0.1));
  }

  @Test
  public void constructorRejectsNonPositiveLinearDriveRadius() {
    assertThrows(
        IllegalArgumentException.class,
        () -> createDynamics(params -> params.linearDriveRadius = 0.0));
  }

  @Test
  public void constructorRejectsNonPositiveDifferentialRadius() {
    assertThrows(
        IllegalArgumentException.class,
        () -> createDynamics(params -> params.differentialArmRadius = 0.0));
  }

  @Test
  public void constructorRejectsNegativeRotorInertia() {
    assertThrows(
        IllegalArgumentException.class,
        () -> createDynamics(params -> params.motorRotorInertia = -0.1));
  }

  @Test
  public void constructorRejectsNullRightMotor() {
    assertThrows(
        IllegalArgumentException.class, () -> createDynamics(params -> params.rightMotor = null));
  }

  @Test
  public void constructorRejectsNullLeftMotor() {
    assertThrows(
        IllegalArgumentException.class, () -> createDynamics(params -> params.leftMotor = null));
  }

  @Test
  public void constructorRejectsNonFiniteGravity() {
    assertThrows(
        IllegalArgumentException.class,
        () -> createDynamics(params -> params.gravity = Double.NaN));
  }

  @Test
  public void constructorRejectsNonFiniteInclination() {
    assertThrows(
        IllegalArgumentException.class,
        () -> createDynamics(params -> params.extensionInclination = Double.POSITIVE_INFINITY));
  }

  @Test
  public void constructorRejectsNonFiniteSensorOffset() {
    assertThrows(
        IllegalArgumentException.class,
        () -> createDynamics(params -> params.sensorOffset = Double.NEGATIVE_INFINITY));
  }
}
