package frc.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.Timer;

/**
 * A PID controller that generates motion using WPILib's {@link ExponentialProfile}. This controller
 * creates smooth exponential transitions to a position goal.
 */
public class ExponentialProfiledPIDController {
  private final PIDController m_controller;
  private ExponentialProfile.Constraints m_constraints;
  private ExponentialProfile m_profile;

  private double m_goal;
  private double m_t0;
  private double m_initial;

  private ExponentialProfile.State m_currentSetpoint = new ExponentialProfile.State();

  private double m_positionTolerance = 0.0;
  private double m_velocityTolerance = Double.POSITIVE_INFINITY;

  /**
   * Constructs a new ExponentialProfiledPIDController.
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param constraints Constraints for the exponential profile (max input, system dynamics)
   */
  public ExponentialProfiledPIDController(
      double kP, double kI, double kD, ExponentialProfile.Constraints constraints) {
    m_controller = new PIDController(kP, kI, kD);
    m_constraints = constraints;
    m_goal = 0.0;
    m_initial = 0.0;
    m_currentSetpoint = new ExponentialProfile.State(m_initial, 0.0);
  }

  /**
   * Sets a new goal for the controller.
   *
   * @param goal The desired target position
   */
  public void setGoal(double goal) {
    m_goal = goal;
    m_t0 = Timer.getFPGATimestamp();
    m_initial = m_currentSetpoint.position;
    m_profile = new ExponentialProfile(m_constraints);
  }

  /** Returns true if the controller is within the position and velocity tolerances of the goal. */
  public boolean atGoal() {
    return Math.abs(m_goal - m_currentSetpoint.position) <= m_positionTolerance
        && Math.abs(m_currentSetpoint.velocity) <= m_velocityTolerance;
  }

  /**
   * Calculates the output of the controller based on the current measurement.
   *
   * @param measurement The current position of the mechanism
   * @return The control output
   */
  public double calculate(double measurement) {
    double currentTime = Timer.getFPGATimestamp();
    double elapsedTime = currentTime - m_t0;

    m_currentSetpoint =
        m_profile.calculate(
            elapsedTime,
            new ExponentialProfile.State(m_initial, 0.0),
            new ExponentialProfile.State(m_goal, 0.0));

    return m_controller.calculate(measurement, m_currentSetpoint.position);
  }

  /**
   * Resets the controller state and internal profile.
   *
   * @param measurement The current position to sync from
   */
  public void reset(double measurement) {
    m_controller.reset();
    m_currentSetpoint = new ExponentialProfile.State(measurement, 0.0);
    m_initial = measurement;
    setGoal(m_goal); // Rebuild profile from new start point
  }

  /**
   * Sets the position and velocity tolerances used by {@link #atGoal()}.
   *
   * @param positionTolerance Max allowable error in position
   * @param velocityTolerance Max allowable velocity
   */
  public void setTolerance(double positionTolerance, double velocityTolerance) {
    m_positionTolerance = positionTolerance;
    m_velocityTolerance = velocityTolerance;
  }

  /** Gets the current setpoint position. */
  public double getSetpointPosition() {
    return m_currentSetpoint.position;
  }

  /** Gets the current setpoint velocity. */
  public double getSetpointVelocity() {
    return m_currentSetpoint.velocity;
  }

  /** Gets the full setpoint state (position and velocity). */
  public ExponentialProfile.State getSetpointState() {
    return m_currentSetpoint;
  }

  /** Returns the current goal position. */
  public double getGoal() {
    return m_goal;
  }

  /** Returns true if the internal PID is within its position tolerance. */
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  /** Returns the internal PIDController. */
  public PIDController getPIDController() {
    return m_controller;
  }
}
