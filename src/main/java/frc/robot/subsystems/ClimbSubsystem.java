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
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Climber;
import frc.robot.Constants.DriveConstants;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;

/** Controls the telescoping climb winch and the servo that deploys the hook. */
public class ClimbSubsystem extends SubsystemBase {
  private SparkMax leftMotor;

  /** PWM servo that deploys or retracts the climb hook. */
  private Servo servo;

  /** Stored servo angle when the hook is deployed. */
  private double servoOpen = 90;

  /** Stored servo angle when the hook is safely stowed. */
  private double servoClosed = 180;

  private SparkClosedLoopController climber;

  private RelativeEncoder leftEnc;

  private SparkMaxConfig leftConfig = new SparkMaxConfig();

  /** Latest requested winch position in native encoder units. */
  private double climberSetpoint = 0;

  /** Desired time (seconds) to move between the in and out setpoints. */
  public double tClimb = 2;

  /** Slew-rate (units/sec) derived from the travel time so the winch accelerates smoothly. */
  public double vSlewrate =
      Math.abs((Climber.climberOutSetpoint - Climber.climberInSetpoint) / tClimb);

  /** Approximate drive distance covered while the winch is moving, used for dashboard context. */
  public double climbDistance =
      (Units.inchesToMeters(20) / DriveConstants.kMaxSpeedMetersPerSecond);

  /** Limits how quickly the climb setpoint changes to protect the hook and winch. */
  private SlewRateLimiter climberSlew = new SlewRateLimiter(vSlewrate);

  private boolean climbing = false;

  /** Dashboard logger used to stream climb telemetry for tuning. */
  private FlytLogger climbDash = new FlytLogger("Climb");

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    leftMotor = new SparkMax(Climber.kLeftClimberMotorId, MotorType.kBrushless);

    climber = leftMotor.getClosedLoopController();

    leftEnc = leftMotor.getEncoder();
    leftEnc.setPosition(0);

    leftConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(60)
        .encoder
        .positionConversionFactor(Climber.kPositionConversion)
        .velocityConversionFactor(Climber.kVelocityConversion);
    leftConfig
        .closedLoop
        .p(Climber.kP, ClosedLoopSlot.kSlot0)
        .i(Climber.kI, ClosedLoopSlot.kSlot0)
        .d(Climber.kD, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1);

    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    servo = new Servo(1);

    setServoOpen();

    climbDash.addDoublePublisher("Climb Pos", true, () -> getClimberPos());
    climbDash.addDoublePublisher("Climb Setpoint", true, () -> getClimberSetpoint());
    climbDash.addDoublePublisher("Climb Servo Pos", true, () -> getServoPos());
    climbDash.addDoublePublisher("Climb Current", true, () -> leftMotor.getOutputCurrent());
  }

  public boolean getClimbing() {
    return climbing;
  }

  public void setClimbing(boolean hold) {
    climbing = hold;
  }

  public void setClimberSpeed(double speed) {
    leftMotor.set(speed);
  }

  public void stopClimberMotor() {
    leftMotor.stopMotor();
  }

  public Command setClimberSpeedCommand(double speed) {
    return this.runOnce(() -> setClimberSpeed(speed));
  }

  public double getClimberPos() {
    return leftEnc.getPosition();
  }

  public double getClimberVel() {
    return leftEnc.getVelocity();
  }

  public double getClimberSetpoint() {
    return climberSetpoint;
  }

  public void setClimberSetpoint(double setpoint) {
    climberSetpoint = setpoint;
  }

  public boolean atClimberSetpoint() {
    return MathUtil.isNear(getClimberPos(), climberSetpoint, 5);
  }

  public boolean climberOut() {
    return (Climber.climberOutSetpoint - 5) < getClimberPos() && getClimberPos() < (Climber.climberInSetpoint + 5);
  }

  public double getServoPos() {
    return servo.getPosition();
  }

  public void setServoPos(double pos) {
    servo.setAngle(pos);
  }

  public void setServoClose() {
    servo.setAngle(servoClosed);
  }

  public void setServoOpen() {
    servo.setAngle(servoOpen);
  }

  @Override
  public void periodic() {
    if (!climbing) {
      // Slew-limit the setpoint to avoid sudden shocks on the climb structure.
      climber.setReference(climberSlew.calculate(climberSetpoint), ControlType.kPosition);
    }
    // This method will be called once per scheduler run
    climbDash.update(Constants.debugMode);
  }
}
