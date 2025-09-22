// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;
import frc.robot.commands.Waits.ElevatorSetWait;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;

// import frc.utils.FLYTLib.FLYTMotorLib.FlytMotorController;
// import frc.utils.FLYTLib.FLYTMotorLib.SparkFlexController;

/** Controls the elevator carriage that raises and lowers the manipulator. */
public class ElevatorSubsystem extends SubsystemBase {

  /** Motion-profiled PID that commands elevator velocity while respecting trapezoid limits. */
  private ProfiledPIDController traPidController =
      new ProfiledPIDController(
          Elevator.kProfiledKp,
          Elevator.kProfiledKi,
          Elevator.kProfiledKd,
          new TrapezoidProfile.Constraints(
              Elevator.kProfiledMaxVelocityMetersPerSecond,
              Elevator.kProfiledMaxAccelerationMetersPerSecondSquared));

  private SparkFlex leftMotor;
  private SparkFlex rightMotor;

  private SparkFlexConfig leftConfig = new SparkFlexConfig();
  private SparkFlexConfig rightConfig = new SparkFlexConfig();

  private SparkClosedLoopController elevator;

  private RelativeEncoder leftEnc;

  /** Dashboard logger used to monitor elevator performance. */
  private FlytLogger elevatorDash = new FlytLogger("Elevator");

  /** Desired carriage height in meters. */
  private double elevatorSetpoint = 0;

  public ElevatorSubsystem() {

    leftMotor = new SparkFlex(Elevator.kLeftElevatorMotorId, MotorType.kBrushless);
    rightMotor = new SparkFlex(Elevator.kRightElevatorMotorId, MotorType.kBrushless);

    elevator = leftMotor.getClosedLoopController();

    leftEnc = leftMotor.getEncoder();
    leftEnc.setPosition(0);

    leftConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Elevator.kCurrentLimitAmps)
        .encoder
        .positionConversionFactor(Elevator.kPositionConversion)
        .velocityConversionFactor(Elevator.kVelocityConversion);
    leftConfig
        .closedLoop
        .p(Elevator.kP, ClosedLoopSlot.kSlot0)
        .i(Elevator.kI, ClosedLoopSlot.kSlot0)
        .d(Elevator.kD, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1);

    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightConfig.follow(Elevator.kLeftElevatorMotorId, true).idleMode(IdleMode.kBrake);

    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    traPidController.reset(elevatorSetpoint);

    elevatorDash.addDoublePublisher("Elevator Position", false, () -> getPosition());
    elevatorDash.addDoublePublisher("Elevator Setpoint", false, () -> getElevatorSetpoint());
    elevatorDash.addBoolPublisher("At Position", false, () -> atPosition());
    elevatorDash.addDoublePublisher("Duty Cycle", true, () -> leftMotor.getAppliedOutput());
    elevatorDash.addDoublePublisher("Voltage", true, () -> leftMotor.getBusVoltage());
    elevatorDash.addDoublePublisher("Current", true, () -> leftMotor.getOutputCurrent());
  }

  /**
   * Change the current setpoint for the Elevator
   *
   * @param setpoint - Desired position for elevator
   */
  public void setElevatorSetpoint(double setpoint) {
    // leftMotor.set(setpoint);
    elevatorSetpoint = setpoint;
  }

  public Command setElevatorSetpointCommand(double setpoint) {
    return new ElevatorSetWait(this, setpoint);
    // return Commands.run(() -> setElevatorSetpoint(setpoint)).until(() ->
    // atPosition());
  }

  public Command incrementElevatorSetpoint(double increment) {
    return Commands.runOnce(() -> elevatorSetpoint += increment);
  }

  public double getElevatorSetpoint() {
    return elevatorSetpoint;
  }

  /**
   * Set speed of elevator (value between -1 and 1)
   *
   * @param power - Speed setting (between -1 and 1)
   */
  public void setSpeed(double power) {
    leftMotor.set(power);
  }

  /**
   * Returns current pos of Elevator
   *
   * @return Pos of Elevator
   */
  public double getPosition() {
    return leftEnc.getPosition();
  }

  /**
   * Determines if Elevator is at current setpoint
   *
   * @return True if at setpoint
   */
  public boolean atPosition() {
    return Math.abs(getPosition() - elevatorSetpoint) <= Elevator.kPositionToleranceMeters;
  }

  @Override
  public void periodic() {
    double velocity = traPidController.calculate(getPosition(), elevatorSetpoint);
    elevator.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    elevatorDash.update(Constants.debugMode);
  }
}
