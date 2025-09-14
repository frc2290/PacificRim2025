package frc.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalJacobian;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.function.BiFunction;

/**
 * Core physics and dynamics for a 2-DOF planar arm driven by a differential motor pair.
 *
 * <p>State x = [extension, extension_dot, theta_meas, theta_dot]'. <br>
 * Input u = [V_R, V_L]'.
 *
 * <p>Notes:
 *
 * <ul>
 *   <li>theta = theta_meas + sensorOffset is the ABSOLUTE angle w.r.t. gravity.
 *   <li>Viscous + smoothed Coulomb friction model.
 *   <li>Rotor inertia term is used as provided.
 * </ul>
 */
public class DifferentialArmDynamics {
  private final DCMotor m_rightMotor;
  private final DCMotor m_leftMotor;

  private final double m_extensionMass;
  private final double m_rotationMass;
  private final double m_rotationInertia;

  private final double m_comOffset;
  private final double m_extensionInclination;
  private final double m_gravity;

  private final double m_extensionViscousDamping;
  private final double m_extensionCoulombFriction;
  private final double m_rotationViscousDamping;
  private final double m_rotationCoulombFriction;

  private final double m_linearDriveRadius;
  private final double m_differentialArmRadius;
  private final double m_sensorOffset;
  private final double m_motorRotorInertia;

  /**
   * Constructs the nonlinear dynamics model for the differential arm.
   *
   * @param extensionMass Mass along the extension axis (kg).
   * @param rotationMass Mass of the rotating link (kg).
   * @param rotationInertia Rotational inertia about the rotation joint (kg·m^2).
   * @param comOffset Distance from rotation joint to link COM (m).
   * @param extensionInclination Inclination of the extension axis relative to gravity (rad).
   * @param gravity Gravitational acceleration (m/s^2).
   * @param extensionViscousDamping Viscous damping on extension (N·s/m).
   * @param extensionCoulombFriction Coulomb friction on extension (N).
   * @param rotationViscousDamping Viscous damping on rotation (N·m·s/rad).
   * @param rotationCoulombFriction Coulomb friction on rotation (N·m).
   * @param rightMotor Right motor model.
   * @param leftMotor Left motor model.
   * @param linearDriveRadius Effective linear drive radius (m).
   * @param differentialArmRadius Effective differential arm radius (m).
   * @param sensorOffset Offset that converts measured theta to absolute theta (rad).
   * @param motorRotorInertia Sum of motor rotor inertias (reflected at motor shafts) (kg·m^2).
   */
  public DifferentialArmDynamics(
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
      double motorRotorInertia) {

    m_rightMotor = rightMotor;
    m_leftMotor = leftMotor;

    m_extensionMass = extensionMass;
    m_rotationMass = rotationMass;
    m_rotationInertia = rotationInertia;

    m_comOffset = comOffset;
    m_extensionInclination = extensionInclination;
    m_gravity = gravity;

    m_extensionViscousDamping = extensionViscousDamping;
    m_extensionCoulombFriction = extensionCoulombFriction;
    m_rotationViscousDamping = rotationViscousDamping;
    m_rotationCoulombFriction = rotationCoulombFriction;

    m_linearDriveRadius = linearDriveRadius;
    m_differentialArmRadius = differentialArmRadius;
    m_sensorOffset = sensorOffset;
    m_motorRotorInertia = motorRotorInertia;
  }

  /**
   * Creates a linearized {@link LinearSystem} model of the arm's dynamics.
   *
   * @param rNominalMeters Extension value for linearization (m).
   * @param thetaNominalRads Absolute rotation angle for linearization (rad).
   * @return Linearized state-space model with C = I, D = 0.
   */
  public LinearSystem<N4, N2, N4> createLinearizedSystem(
      double rNominalMeters, double thetaNominalRads) {

    Matrix<N4, N1> xOp = VecBuilder.fill(rNominalMeters, 0.0, thetaNominalRads, 0.0);
    Matrix<N2, N1> uOp = calculateFeedforward(xOp);

    BiFunction<Matrix<N4, N1>, Matrix<N2, N1>, Matrix<N4, N1>> f = this::dynamics;

    Matrix<N4, N4> A = NumericalJacobian.numericalJacobianX(Nat.N4(), Nat.N4(), f, xOp, uOp);
    Matrix<N4, N2> B = NumericalJacobian.numericalJacobianU(Nat.N4(), Nat.N2(), f, xOp, uOp);

    return new LinearSystem<>(A, B, Matrix.eye(Nat.N4()), new Matrix<>(Nat.N4(), Nat.N2()));
  }

  /**
   * Feedforward voltages required to statically hold the arm (ω = 0).
   *
   * @param x State vector [extension, extension_dot, theta_meas, theta_dot]'.
   * @return Feedforward voltages [V_R, V_L]'.
   */
  public Matrix<N2, N1> calculateFeedforward(Matrix<N4, N1> x) {
    double thetaAbs = x.get(2, 0) + m_sensorOffset;

    double G1 = (m_extensionMass + m_rotationMass) * m_gravity * Math.sin(m_extensionInclination);
    double G2 = m_rotationMass * m_gravity * m_comOffset * Math.cos(thetaAbs);

    double fExtReq = G1;
    double tauThetaReq = G2;

    double tauR =
        (fExtReq * m_linearDriveRadius
                - tauThetaReq * m_linearDriveRadius / m_differentialArmRadius)
            / 2.0;
    double tauL =
        (fExtReq * m_linearDriveRadius
                + tauThetaReq * m_linearDriveRadius / m_differentialArmRadius)
            / 2.0;

    double vR = m_rightMotor.getVoltage(tauR, 0.0);
    double vL = m_leftMotor.getVoltage(tauL, 0.0);

    return VecBuilder.fill(vR, vL);
  }

  /**
   * Nonlinear plant ẋ = f(x, u).
   *
   * @param x State vector [extension, extension_dot, theta_meas, theta_dot]'.
   * @param u Input vector [V_R, V_L]'.
   * @return Derivative [extension_dot, extension_ddot, theta_dot, theta_ddot]'.
   */
  public Matrix<N4, N1> dynamics(Matrix<N4, N1> x, Matrix<N2, N1> u) {
    double rDot = x.get(1, 0);
    double thetaMeas = x.get(2, 0);
    double thetaDot = x.get(3, 0);

    double vR = u.get(0, 0);
    double vL = u.get(1, 0);

    double thetaAbs = thetaMeas + m_sensorOffset;

    // Differential kinematics → motor speeds
    double omegaR =
        (rDot / m_linearDriveRadius) - (m_differentialArmRadius / m_linearDriveRadius) * thetaDot;
    double omegaL =
        (rDot / m_linearDriveRadius) + (m_differentialArmRadius / m_linearDriveRadius) * thetaDot;

    // Motor currents/torques from applied voltages
    double iR = m_rightMotor.getCurrent(omegaR, vR);
    double iL = m_leftMotor.getCurrent(omegaL, vL);
    double tauR = m_rightMotor.getTorque(iR);
    double tauL = m_leftMotor.getTorque(iL);

    // Back to generalized forces
    double fExt = (tauR + tauL) / m_linearDriveRadius;
    double tauTheta = (m_differentialArmRadius / m_linearDriveRadius) * (tauL - tauR);

    // Shorthands
    double sPhiTheta = Math.sin(m_extensionInclination - thetaAbs);
    double cPhiTheta = Math.cos(m_extensionInclination - thetaAbs);
    double cTheta = Math.cos(thetaAbs);

    // Inertia (with rotor inertia reflection)
    double m11 =
        m_extensionMass
            + m_rotationMass
            + m_motorRotorInertia / (m_linearDriveRadius * m_linearDriveRadius);
    double m12 = m_rotationMass * m_comOffset * sPhiTheta;
    double m22 =
        m_rotationMass * m_comOffset * m_comOffset
            + m_rotationInertia
            + (m_motorRotorInertia * m_differentialArmRadius * m_differentialArmRadius)
                / (m_linearDriveRadius * m_linearDriveRadius);

    // Coriolis/centrifugal, gravity, damping/friction
    double c1 = -m_rotationMass * m_comOffset * cPhiTheta * thetaDot * thetaDot;
    double g1 = (m_extensionMass + m_rotationMass) * m_gravity * Math.sin(m_extensionInclination);
    double g2 = m_rotationMass * m_gravity * m_comOffset * cTheta;

    double d1 =
        m_extensionViscousDamping * rDot + m_extensionCoulombFriction * Math.tanh(100.0 * rDot);
    double d2 =
        m_rotationViscousDamping * thetaDot
            + m_rotationCoulombFriction * Math.tanh(100.0 * thetaDot);

    double rhs1 = fExt - c1 - g1 - d1;
    double rhs2 = tauTheta - g2 - d2;

    // Solve M * [r_ddot; theta_ddot] = rhs
    double det = m11 * m22 - m12 * m12;
    det = Math.abs(det) < 1e-9 ? 1e-9 : det; // numeric guard only
    double invDet = 1.0 / det;

    double rDDot = invDet * (m22 * rhs1 - m12 * rhs2);
    double thetaDDot = invDet * (-m12 * rhs1 + m11 * rhs2);

    return VecBuilder.fill(rDot, rDDot, thetaDot, thetaDDot);
  }

  /** Right motor current (A), signed (regen negative). */
  public double getRightMotorCurrentAmps(Matrix<N4, N1> x, Matrix<N2, N1> u) {
    double rDot = x.get(1, 0);
    double thetaDot = x.get(3, 0);
    double omegaR =
        (rDot / m_linearDriveRadius) - (m_differentialArmRadius / m_linearDriveRadius) * thetaDot;
    return m_rightMotor.getCurrent(omegaR, u.get(0, 0));
  }

  /** Left motor current (A), signed (regen negative). */
  public double getLeftMotorCurrentAmps(Matrix<N4, N1> x, Matrix<N2, N1> u) {
    double rDot = x.get(1, 0);
    double thetaDot = x.get(3, 0);
    double omegaL =
        (rDot / m_linearDriveRadius) + (m_differentialArmRadius / m_linearDriveRadius) * thetaDot;
    return m_leftMotor.getCurrent(omegaL, u.get(1, 0));
  }

  /**
   * Total current (A) defined as the sum of absolute per-motor currents. Useful for PDH/battery
   * models.
   */
  public double getTotalCurrentAbsAmps(Matrix<N4, N1> x, Matrix<N2, N1> u) {
    double iR = getRightMotorCurrentAmps(x, u);
    double iL = getLeftMotorCurrentAmps(x, u);
    return Math.abs(iR) + Math.abs(iL);
  }

  /**
   * @deprecated Use {@link #getTotalCurrentAbsAmps(Matrix, Matrix)} or the per-motor accessors.
   */
  @Deprecated
  public double getCurrentDrawAmps(Matrix<N4, N1> x, Matrix<N2, N1> u) {
    return getTotalCurrentAbsAmps(x, u);
  }
}
