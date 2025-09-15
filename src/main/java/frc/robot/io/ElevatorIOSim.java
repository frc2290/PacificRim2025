package frc.robot.io;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.Elevator;

/** MapleSim-backed simulation for the elevator. */
public class ElevatorIOSim implements ElevatorIO {
  private final SparkFlex leftMotor;
  private final SparkFlex rightMotor;
  private final SparkFlexSim leftSim;
  private final SparkFlexSim rightSim;
  private final SparkRelativeEncoderSim leftEncoderSim;
  private final ElevatorSim elevatorSim;
  private double appliedVolts;

  public ElevatorIOSim() {
    leftMotor = new SparkFlex(Elevator.kLeftElevatorMotorId, MotorType.kBrushless);
    rightMotor = new SparkFlex(Elevator.kRightElevatorMotorId, MotorType.kBrushless);

    leftSim = new SparkFlexSim(leftMotor, DCMotor.getNeoVortex(1));
    rightSim = new SparkFlexSim(rightMotor, DCMotor.getNeoVortex(1));
    leftEncoderSim = leftSim.getRelativeEncoderSim();
    elevatorSim =
        new ElevatorSim(
            DCMotor.getNeoVortex(2),
            1.0 / Elevator.kSimGearing,
            Elevator.kSimCarriageMassKg,
            Elevator.kSimDrumRadiusMeters,
            Elevator.kSimMinHeightMeters,
            Elevator.kSimMaxHeightMeters,
            true,
            0.0);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public double getPositionMeters() {
    return elevatorSim.getPositionMeters();
  }

  @Override
  public double getVelocityMetersPerSec() {
    return elevatorSim.getVelocityMetersPerSecond();
  }

  @Override
  public double getCurrentDrawAmps() {
    return elevatorSim.getCurrentDrawAmps();
  }

  @Override
  public void update() {
    double busVoltage = RobotController.getBatteryVoltage();
    if (busVoltage <= 0.0) {
      return;
    }

    elevatorSim.setInputVoltage(appliedVolts);
    elevatorSim.update(0.02);

    double motorOmega =
        elevatorSim.getVelocityMetersPerSecond()
            / Elevator.kSimDrumRadiusMeters
            / Elevator.kSimGearing;
    double motorAngle =
        elevatorSim.getPositionMeters() / Elevator.kSimDrumRadiusMeters / Elevator.kSimGearing;
    double rotorRPM = motorOmega * 60.0 / (2 * Math.PI);

    leftSim.iterate(rotorRPM, busVoltage, 0.02);
    leftEncoderSim.setPosition(motorAngle);
    leftEncoderSim.setVelocity(rotorRPM);

    rightSim.iterate(-rotorRPM, busVoltage, 0.02);
  }
}
