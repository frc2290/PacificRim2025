package frc.robot.subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;
import frc.robot.commands.Waits.ElevatorSetWait;
import frc.robot.io.ElevatorIO;

/** Closed-loop elevator subsystem using an IO abstraction. */
public class ElevatorSubsystem extends SubsystemBase {
  private final ElevatorIO io;
  private final ProfiledPIDController controller =
      new ProfiledPIDController(64, 0, 1, new TrapezoidProfile.Constraints(2.5, 9));
  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(Elevator.kS, Elevator.kG, Elevator.kV, Elevator.kA);

  private double setpointMeters = 0.0;

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    controller.reset(setpointMeters);
  }

  /** Change the desired elevator position in meters. */
  public void setElevatorSetpoint(double setpoint) {
    setpointMeters = setpoint;
  }

  /** Command to move the elevator to the specified setpoint. */
  public Command setElevatorSetpointCommand(double setpoint) {
    return new ElevatorSetWait(this, setpoint);
  }

  /** Increment the current elevator setpoint by the given amount. */
  public Command incrementElevatorSetpoint(double increment) {
    return Commands.runOnce(() -> setpointMeters += increment);
  }

  /** Get the current setpoint. */
  public double getElevatorSetpoint() {
    return setpointMeters;
  }

  /** Return the measured elevator position in meters. */
  public double getPosition() {
    return io.getPositionMeters();
  }

  /** Determine if the elevator is within 4 cm of its setpoint. */
  public boolean atPosition() {
    return Math.abs(getPosition() - setpointMeters) <= 0.04;
  }

  /** Combined current draw of elevator motors in amps. */
  public double getCurrentDraw() {
    return io.getCurrentDrawAmps();
  }

  @Override
  public void periodic() {
    io.update();
    double velocity = controller.calculate(getPosition(), setpointMeters);
    double volts = feedforward.calculate(velocity);
    io.setVoltage(volts);
  }
}
