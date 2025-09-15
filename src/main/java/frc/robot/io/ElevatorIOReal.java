package frc.robot.io;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.Constants.Elevator;

/** Real hardware implementation of the elevator IO. */
public class ElevatorIOReal implements ElevatorIO {
  private final SparkFlex leftMotor;
  private final SparkFlex rightMotor;
  private final RelativeEncoder leftEncoder;
  private final SparkClosedLoopController controller;

  public ElevatorIOReal() {
    leftMotor = new SparkFlex(Elevator.kLeftElevatorMotorId, MotorType.kBrushless);
    rightMotor = new SparkFlex(Elevator.kRightElevatorMotorId, MotorType.kBrushless);

    controller = leftMotor.getClosedLoopController();
    leftEncoder = leftMotor.getEncoder();
    leftEncoder.setPosition(0.0);

    SparkFlexConfig leftConfig = new SparkFlexConfig();
    leftConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(50);
    leftConfig
        .encoder
        .positionConversionFactor(Elevator.kPositionConversion)
        .velocityConversionFactor(Elevator.kVelocityConversion);
    leftConfig.closedLoop.p(Elevator.kP).i(Elevator.kI).d(Elevator.kD).outputRange(-1, 1);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig rightConfig = new SparkFlexConfig();
    rightConfig.follow(Elevator.kLeftElevatorMotorId, true).idleMode(IdleMode.kBrake);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setVoltage(double volts) {
    controller.setReference(volts, ControlType.kVoltage);
  }

  @Override
  public double getPositionMeters() {
    return leftEncoder.getPosition();
  }

  @Override
  public double getVelocityMetersPerSec() {
    return leftEncoder.getVelocity();
  }

  @Override
  public double getCurrentDrawAmps() {
    return leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent();
  }
}
