package frc.robot.io;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.Manipulator;

/** Simulation implementation of the manipulator IO. */
public class ManipulatorIOSim implements ManipulatorIO {
  private final SparkFlex motor;
  private final SparkFlexSim motorSim;
  private final SparkRelativeEncoderSim encoderSim;
  private final SparkLimitSwitchSim limitSwitchSim;
  private final DCMotorSim motorModel;
  private final Timer coralTimer = new Timer();
  private final Debouncer coralDebounce = new Debouncer(0.05);
  private double requestedVolts;
  private double appliedVolts;
  private boolean coralPresent = true;
  private boolean algaePresent;

  public ManipulatorIOSim() {
    motor = new SparkFlex(Manipulator.kManipulatorMotorId, MotorType.kBrushless);
    motorSim = new SparkFlexSim(motor, DCMotor.getNEO(1));
    encoderSim = motorSim.getRelativeEncoderSim();
    limitSwitchSim = new SparkLimitSwitchSim(motor, true);
    motorModel =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, 1.0), DCMotor.getNEO(1));
  }

  @Override
  public void setVoltage(double volts) {
    requestedVolts = volts;
  }

  @Override
  public double getPositionRotations() {
    return encoderSim.getPosition();
  }

  @Override
  public double getWristPosition() {
    return encoderSim.getPosition();
  }

  @Override
  public double getVelocityRPM() {
    return encoderSim.getVelocity();
  }

  @Override
  public double getCurrentAmps() {
    return motorSim.getMotorCurrent();
  }

  @Override
  public boolean seesCoral() {
    return coralDebounce.calculate(limitSwitchSim.getPressed());
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
    encoderSim.setPosition(0.0);
  }

  @Override
  public void update() {
    double busVoltage = RobotController.getBatteryVoltage();
    double availableVoltage = Math.abs(busVoltage);

    if (availableVoltage > 1e-3) {
      appliedVolts = MathUtil.clamp(requestedVolts, -availableVoltage, availableVoltage);
      motorSim.setAppliedOutput(appliedVolts / busVoltage);
    } else {
      appliedVolts = 0.0;
      motorSim.setAppliedOutput(0.0);
    }

    motorModel.setInputVoltage(appliedVolts);
    motorModel.update(0.02);
    double rotorRPM = motorModel.getAngularVelocityRPM();
    motorSim.iterate(rotorRPM, availableVoltage, 0.02);
    encoderSim.setPosition(motorModel.getAngularPositionRotations());
    encoderSim.setVelocity(rotorRPM);

    if (!coralPresent && Math.abs(appliedVolts) > 0.05) {
      if (!coralTimer.isRunning()) {
        coralTimer.restart();
        limitSwitchSim.setPressed(true);
      } else if (coralTimer.hasElapsed(0.1)) {
        coralPresent = true;
        limitSwitchSim.setPressed(false);
        coralTimer.stop();
      }
    } else {
      limitSwitchSim.setPressed(false);
      coralTimer.stop();
    }
  }
}
