package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import org.ejml.simple.SimpleMatrix;

/**
 * High level simulator for a swerve drivetrain. This class aggregates a set of {@link
 * SwerveModuleSim} instances and enforces the non-holonomic rolling constraints to produce
 * realistic robot motion.
 */
public class SwerveDriveSim {
  private final List<SwerveModuleSim> m_modules;
  private final Translation2d[] m_modulePos;
  private final double m_mass;
  private final double m_inertiaZ;
  private final double m_linearDamping;
  private final double m_angularDamping;
  private final double m_beta;

  private Pose2d m_pose = new Pose2d();
  private double m_vx;
  private double m_vy;
  private double m_omega;
  // Stored unconstrained speeds from the latest update step for debugging.
  private double m_vxUn;
  private double m_vyUn;
  private double m_omegaUn;
  // Acceleration delta applied by the non-holonomic projection.
  private double m_axProj;
  private double m_ayProj;
  private double m_alphaProj;

  /**
   * Constructs a new swerve drive simulator.
   *
   * @param modules list of module simulators
   * @param modulePos array of module positions relative to robot center
   * @param mass robot mass in kilograms
   * @param inertiaZ robot yaw moment of inertia about the center (kg*m^2)
   * @param linearDamping linear damping coefficient (N*s/m)
   * @param angularDamping angular damping coefficient (N*m*s/rad)
   * @param beta blending factor to soften constraints (0-1). 1.0 is fully rigid.
   */
  public SwerveDriveSim(
      List<SwerveModuleSim> modules,
      Translation2d[] modulePos,
      double mass,
      double inertiaZ,
      double linearDamping,
      double angularDamping,
      double beta) {
    m_modules = modules;
    m_modulePos = modulePos;
    m_mass = mass;
    m_inertiaZ = inertiaZ;
    m_linearDamping = linearDamping;
    m_angularDamping = angularDamping;
    m_beta = beta;
  }

  /** Overloaded constructor with default beta of 1.0 (fully rigid). */
  public SwerveDriveSim(
      List<SwerveModuleSim> modules,
      Translation2d[] modulePos,
      double mass,
      double inertiaZ,
      double linearDamping,
      double angularDamping) {
    this(modules, modulePos, mass, inertiaZ, linearDamping, angularDamping, 1.0);
  }

  /** Default damping of zero. */
  public SwerveDriveSim(
      List<SwerveModuleSim> modules, Translation2d[] modulePos, double mass, double inertiaZ) {
    this(modules, modulePos, mass, inertiaZ, 0.0, 0.0, 1.0);
  }

  /** Returns the robot's current pose. */
  public Pose2d getPose() {
    return m_pose;
  }

  /** Returns the current chassis speeds in the body frame. */
  public ChassisSpeeds getSpeeds() {
    return new ChassisSpeeds(m_vx, m_vy, m_omega);
  }

  /** Returns the unconstrained chassis speeds before applying non-holonomic projection. */
  public ChassisSpeeds getUnconstrainedSpeeds() {
    return new ChassisSpeeds(m_vxUn, m_vyUn, m_omegaUn);
  }

  /** Returns the chassis acceleration change caused by the projection step. */
  public ChassisSpeeds getProjectionAccel() {
    return new ChassisSpeeds(m_axProj, m_ayProj, m_alphaProj);
  }

  /** Sets the robot's current pose. */
  public void setPose(Pose2d pose) {
    m_pose = pose;
  }

  /** Sets the robot's velocity state in the body frame. */
  public void setSpeeds(ChassisSpeeds speeds) {
    m_vx = speeds.vxMetersPerSecond;
    m_vy = speeds.vyMetersPerSecond;
    m_omega = speeds.omegaRadiansPerSecond;
  }

  /**
   * Performs one simulation tick by commanding each module with the supplied setpoints and
   * integrating the resulting chassis motion.
   *
   * @param busVoltage battery voltage supplied to the controllers
   * @param driveSetpoints array of drive velocity setpoints (m/s) per module
   * @param steerSetpoints array of steer angle setpoints (rad) per module
   * @param dt timestep in seconds
   */
  public void update(
      double busVoltage, double[] driveSetpoints, double[] steerSetpoints, double dt) {
    int n = m_modules.size();
    if (driveSetpoints.length != n || steerSetpoints.length != n) {
      throw new IllegalArgumentException("Setpoint array length must match module count");
    }

    SwerveModuleSim.ModuleForce[] forces = new SwerveModuleSim.ModuleForce[n];
    for (int i = 0; i < n; i++) {
      forces[i] =
          m_modules
              .get(i)
              .update(
                  driveSetpoints[i],
                  steerSetpoints[i],
                  busVoltage,
                  m_vx,
                  m_vy,
                  m_omega,
                  m_modulePos[i],
                  dt);
    }

    update(forces, busVoltage, dt);
  }

  /**
   * Performs one simulation tick using precomputed module forces.
   *
   * @param forces array of longitudinal forces from each module
   * @param busVoltage current battery voltage for sensor mirroring
   * @param dt timestep in seconds
   */
  public void update(SwerveModuleSim.ModuleForce[] forces, double busVoltage, double dt) {
    int n = m_modules.size();
    if (forces.length != n) {
      throw new IllegalArgumentException("Force array length must match module count");
    }

    // Step 3: integrate for unconstrained twist
    double fx = 0.0;
    double fy = 0.0;
    double tau = 0.0;
    for (int i = 0; i < n; i++) {
      fx += forces[i].fx;
      fy += forces[i].fy;
      tau += m_modulePos[i].getX() * forces[i].fy - m_modulePos[i].getY() * forces[i].fx;
    }
    fx -= m_linearDamping * m_vx;
    fy -= m_linearDamping * m_vy;
    tau -= m_angularDamping * m_omega;

    double vxUn = m_vx + (fx / m_mass) * dt;
    double vyUn = m_vy + (fy / m_mass) * dt;
    double omegaUn = m_omega + (tau / m_inertiaZ) * dt;

    m_vxUn = vxUn;
    m_vyUn = vyUn;
    m_omegaUn = omegaUn;

    SimpleMatrix u = new SimpleMatrix(3, 1, true, vxUn, vyUn, omegaUn);

    // Step 4: build constraint matrix A
    SimpleMatrix A = new SimpleMatrix(n, 3);
    for (int i = 0; i < n; i++) {
      double theta = m_modules.get(i).getAzimuth();
      double nx = -Math.sin(theta);
      double ny = Math.cos(theta);
      double rx = m_modulePos[i].getX();
      double ry = m_modulePos[i].getY();
      A.set(i, 0, nx);
      A.set(i, 1, ny);
      A.set(i, 2, rx * ny - ry * nx);
    }

    SimpleMatrix Winv = SimpleMatrix.diag(1.0 / m_mass, 1.0 / m_mass, 1.0 / m_inertiaZ);
    SimpleMatrix M = A.mult(Winv).mult(A.transpose()).plus(SimpleMatrix.identity(n).scale(1e-9));
    SimpleMatrix vProj = u.minus(Winv.mult(A.transpose()).mult(M.invert()).mult(A).mult(u));

    // Blend unconstrained and fully constrained velocities to soften constraints
    SimpleMatrix vFinal = u.scale(1.0 - m_beta).plus(vProj.scale(m_beta));

    double vx = vFinal.get(0);
    double vy = vFinal.get(1);
    double omega = vFinal.get(2);

    // Record the acceleration change introduced by the projection step
    m_axProj = (vx - vxUn) / dt;
    m_ayProj = (vy - vyUn) / dt;
    m_alphaProj = (omega - omegaUn) / dt;

    m_vx = vx;
    m_vy = vy;
    m_omega = omega;

    // Step 5: update pose
    m_pose = m_pose.exp(new Twist2d(m_vx * dt, m_vy * dt, m_omega * dt));

    // Step 5b: mirror sensors for each module using final constrained twist
    for (int i = 0; i < n; i++) {
      double vcx = m_vx - m_omega * m_modulePos[i].getY();
      double vcy = m_vy + m_omega * m_modulePos[i].getX();
      double theta = m_modules.get(i).getAzimuth();
      double tX = Math.cos(theta);
      double tY = Math.sin(theta);
      double vRoll = vcx * tX + vcy * tY;
      m_modules.get(i).updateDriveSensor(vRoll, dt, busVoltage);
    }
  }

  /** Returns the robot's current yaw rotation. */
  public Rotation2d getRotation2d() {
    return m_pose.getRotation();
  }
}
