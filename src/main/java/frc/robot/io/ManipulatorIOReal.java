package frc.robot.io;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Constants.Manipulator;

/** Real hardware implementation of the manipulator IO. */
public class ManipulatorIOReal implements ManipulatorIO {
  private final SparkFlex motor;
  private final SparkAbsoluteEncoder absEncoder;
  private final RelativeEncoder relEncoder;
  private final SparkLimitSwitch limitSwitch;
  private final Debouncer coralDebounce = new Debouncer(0.05);
  private boolean coralPresent = true;
  private boolean algaePresent;

  public ManipulatorIOReal() {
    motor = new SparkFlex(Manipulator.kManipulatorMotorId, MotorType.kBrushless);
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    absEncoder = motor.getAbsoluteEncoder();
    relEncoder = motor.getEncoder();
    limitSwitch = motor.getForwardLimitSwitch();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public double getPositionRotations() {
    return relEncoder.getPosition();
  }

  @Override
  public double getWristPosition() {
    return absEncoder.getPosition();
  }

  @Override
  public double getVelocityRPM() {
    return relEncoder.getVelocity();
  }

  @Override
  public double getCurrentAmps() {
    return motor.getOutputCurrent();
  }

  @Override
  public boolean seesCoral() {
    return coralDebounce.calculate(limitSwitch.isPressed());
  }

  @Override
  public boolean hasCoral() {
    return coralPresent;
  }

  @Override
  public void setCoral(boolean coral) {
    coralPresent = coral;
  }

  @Override
  public boolean hasAlgae() {
    return algaePresent;
  }

  @Override
  public void setAlgae(boolean algae) {
    algaePresent = algae;
  }

  @Override
  public void resetPosition() {
    relEncoder.setPosition(0.0);
  }
}
