package frc.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

/**
 * Nonlinear simulation model for a 2-DOF planar arm driven by a differential motor pair.
 *
 * <p>Wrapper around {@link DifferentialArmDynamics} providing the simulation loop, state
 * management, and physical limit handling. Uses RK4 integration.
 *
 * <p>State x = [extension, extension_dot, theta_meas, theta_dot]'. <br>
 * Input u = [V_R, V_L]'.
 *
 * <p>Notes:
 *
 * <ul>
 *   <li>LinearSystem shell is built at construction using a linearization around (r_mid,
 *       startingTheta_rads). Outputs map x directly (C = I).
 *   <li>Voltage limits and battery modeling are intentionally NOT handled here.
 * </ul>
 */
public class DifferentialArmSim extends LinearSystemSim<N4, N2, N4> {
  private final DifferentialArmDynamics m_dynamics;

  // Physical limits
  private final double m_minExtensionMeters;
  private final double m_maxExtensionMeters;
  private final double m_minThetaRads;
  private final double m_maxThetaRads;

  /** Public constructor that builds the dynamics from physical parameters. */
  public DifferentialArmSim(
      double extensionMass,
      double rotationMass,
      double rotationInertia,
      double comOffset,
      double extensionInclination,
      double gravity,
      double extensionViscousDamping,
      double extensionCoulombFriction,
      double rotationViscousDamping,
      double rotationCoulombFriction,
      DCMotor rightMotor,
      DCMotor leftMotor,
      double linearDriveRadius,
      double differentialArmRadius,
      double sensorOffset,
      double motorRotorInertia,
      double minExtensionMeters,
      double maxExtensionMeters,
      double minThetaRads,
      double maxThetaRads,
      double startingExtensionMeters,
      double startingThetaRads) {

    this(
        new DifferentialArmDynamics(
            extensionMass,
            rotationMass,
            rotationInertia,
            comOffset,
            extensionInclination,
            gravity,
            extensionViscousDamping,
            extensionCoulombFriction,
            rotationViscousDamping,
            rotationCoulombFriction,
            rightMotor,
            leftMotor,
            linearDriveRadius,
            differentialArmRadius,
            sensorOffset,
            motorRotorInertia),
        minExtensionMeters,
        maxExtensionMeters,
        minThetaRads,
        maxThetaRads,
        startingExtensionMeters,
        startingThetaRads);
  }

  /** Private constructor to accept a prebuilt dynamics object. */
  private DifferentialArmSim(
      DifferentialArmDynamics dynamics,
      double minExtensionMeters,
      double maxExtensionMeters,
      double minThetaRads,
      double maxThetaRads,
      double startingExtensionMeters,
      double startingThetaRads) {

    // Linearize at mid-extension and provided starting absolute theta
    super(
        dynamics.createLinearizedSystem(
            (minExtensionMeters + maxExtensionMeters) / 2.0, startingThetaRads));

    m_dynamics = dynamics;
    m_minExtensionMeters = minExtensionMeters;
    m_maxExtensionMeters = maxExtensionMeters;
    m_minThetaRads = minThetaRads;
    m_maxThetaRads = maxThetaRads;

    // Initial state
    setState(startingExtensionMeters, 0.0, startingThetaRads, 0.0);
  }

  @Override
  protected Matrix<N4, N1> updateX(Matrix<N4, N1> x, Matrix<N2, N1> u, double dt) {
    Matrix<N4, N1> nextX = NumericalIntegration.rk4(m_dynamics::dynamics, x, u, dt);

    double rNext = nextX.get(0, 0);
    double thetaNext = nextX.get(2, 0);

    // Clamp at physical limits and zero velocities on impact
    if (rNext < m_minExtensionMeters) {
      nextX.set(0, 0, m_minExtensionMeters);
      nextX.set(1, 0, 0.0);
    } else if (rNext > m_maxExtensionMeters) {
      nextX.set(0, 0, m_maxExtensionMeters);
      nextX.set(1, 0, 0.0);
    }

    if (thetaNext < m_minThetaRads) {
      nextX.set(2, 0, m_minThetaRads);
      nextX.set(3, 0, 0.0);
    } else if (thetaNext > m_maxThetaRads) {
      nextX.set(2, 0, m_maxThetaRads);
      nextX.set(3, 0, 0.0);
    }

    return nextX;
  }

  // --- Public API ---

  public void setState(
      double extensionMeters, double extensionDotMps, double thetaRads, double thetaDotRps) {
    double r = MathUtil.clamp(extensionMeters, m_minExtensionMeters, m_maxExtensionMeters);
    double th = MathUtil.clamp(thetaRads, m_minThetaRads, m_maxThetaRads);
    setState(VecBuilder.fill(r, extensionDotMps, th, thetaDotRps));
  }

  /** Pass-through; voltage limits are handled externally. */
  public void setInputVoltage(double rightVolts, double leftVolts) {
    setInput(VecBuilder.fill(rightVolts, leftVolts));
  }

  public boolean hasHitMinExtension() {
    return getExtensionPositionMeters() <= m_minExtensionMeters;
  }

  public boolean hasHitMaxExtension() {
    return getExtensionPositionMeters() >= m_maxExtensionMeters;
  }

  public boolean hasHitMinAngle() {
    return getRotationAngleRads() <= m_minThetaRads;
  }

  public boolean hasHitMaxAngle() {
    return getRotationAngleRads() >= m_maxThetaRads;
  }

  public double getExtensionPositionMeters() {
    return getOutput(0);
  }

  public double getExtensionVelocityMetersPerSec() {
    return getOutput(1);
  }

  public double getRotationAngleRads() {
    return getOutput(2);
  }

  public double getRotationVelocityRadsPerSec() {
    return getOutput(3);
  }

  /** Right motor current (A), signed. */
  public double getRightCurrentAmps() {
    return m_dynamics.getRightMotorCurrentAmps(m_x, m_u);
  }

  /** Left motor current (A), signed. */
  public double getLeftCurrentAmps() {
    return m_dynamics.getLeftMotorCurrentAmps(m_x, m_u);
  }

  /** Total current as sum of absolute motor currents (A). */
  public double getTotalCurrentAbsAmps() {
    return m_dynamics.getTotalCurrentAbsAmps(m_x, m_u);
  }

  /**
   * @deprecated Use {@link #getTotalCurrentAbsAmps()} or per-motor accessors.
   */
  @Deprecated
  public double getCurrentDrawAmps() {
    return getTotalCurrentAbsAmps();
  }
}
