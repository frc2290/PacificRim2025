package frc.robot.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DifferentialArm;
import frc.utils.DifferentialArmSim;

/**
 * Simulation-backed implementation of the differential arm IO.
 *
 * <p>Wraps the existing {@link DifferentialArmSim} model so the subsystem can run unmodified logic
 * while still exercising the detailed physics simulation used before the IO abstraction was
 * introduced.
 */
public class DifferentialArmIOSim implements DifferentialArmIO {
  private final DifferentialArmSim sim;

  // Desired spool velocities (mm/s) coming from the subsystem
  private double leftSetpoint;
  private double rightSetpoint;

  // Cached state exposed through the IO API (native units)
  private double leftPos;
  private double rightPos;
  private double leftVel;
  private double rightVel;

  private double leftCurrent;
  private double rightCurrent;

  public DifferentialArmIOSim() {
    sim =
        new DifferentialArmSim(
            DifferentialArm.kSimExtensionMassKg,
            DifferentialArm.kSimRotationMassKg,
            DifferentialArm.kSimRotationInertiaKgM2,
            DifferentialArm.kSimComOffsetMeters,
            DifferentialArm.kSimExtensionInclinationRads,
            DifferentialArm.kSimGravity,
            DifferentialArm.kSimExtensionViscousDamping,
            DifferentialArm.kSimExtensionCoulombFriction,
            DifferentialArm.kSimRotationViscousDamping,
            DifferentialArm.kSimRotationCoulombFriction,
            DCMotor.getNEO(1),
            DCMotor.getNEO(1),
            DifferentialArm.kSimLinearDriveRadiusMeters,
            DifferentialArm.kSimDifferentialArmRadiusMeters,
            DifferentialArm.kSimSensorOffsetRads,
            DifferentialArm.kSimMotorRotorInertia,
            DifferentialArm.kSimMinExtensionMeters,
            DifferentialArm.kSimMaxExtensionMeters,
            DifferentialArm.kSimMinThetaRads,
            DifferentialArm.kSimMaxThetaRads,
            DifferentialArm.kSimStartingExtensionMeters,
            DifferentialArm.kSimStartingThetaRads);
  }

  @Override
  public void setArmVelocitySetpoints(double left, double right) {
    leftSetpoint = left;
    rightSetpoint = right;
  }

  @Override
  public double getLeftPosition() {
    return leftPos;
  }

  @Override
  public double getRightPosition() {
    return rightPos;
  }

  @Override
  public double getLeftVelocity() {
    return leftVel;
  }

  @Override
  public double getRightVelocity() {
    return rightVel;
  }

  @Override
  public double getLeftCurrentAmps() {
    return leftCurrent;
  }

  @Override
  public double getRightCurrentAmps() {
    return rightCurrent;
  }

  @Override
  public void update() {
    // Convert desired spool velocity (mm/s) to motor rad/s
    double leftMotorRadPerSec =
        (leftSetpoint / 1000.0) / DifferentialArm.kSimLinearDriveRadiusMeters;
    double rightMotorRadPerSec =
        (rightSetpoint / 1000.0) / DifferentialArm.kSimLinearDriveRadiusMeters;

    // Approximate voltage needed using motor Kv (rad/s per volt)
    double kv = DCMotor.getNEO(1).KvRadPerSecPerVolt;
    double busVoltage = Math.abs(RobotController.getBatteryVoltage());
    double leftVolts = MathUtil.clamp(leftMotorRadPerSec / kv, -busVoltage, busVoltage);
    double rightVolts = MathUtil.clamp(rightMotorRadPerSec / kv, -busVoltage, busVoltage);

    // DifferentialArmSim expects (right, left)
    sim.setInputVoltage(rightVolts, leftVolts);
    sim.update(0.02);

    double extMeters = sim.getExtensionPositionMeters();
    double rotRads = sim.getRotationAngleRads();
    double extVelMps = sim.getExtensionVelocityMetersPerSec();
    double rotVelRps = sim.getRotationVelocityRadsPerSec();

    // Convert to native units (mm and mm/s)
    double leftRad =
        extMeters / DifferentialArm.kSimLinearDriveRadiusMeters
            - rotRads / DifferentialArm.kSimDifferentialArmRadiusMeters;
    double rightRad =
        extMeters / DifferentialArm.kSimLinearDriveRadiusMeters
            + rotRads / DifferentialArm.kSimDifferentialArmRadiusMeters;

    double leftRadPerSec =
        extVelMps / DifferentialArm.kSimLinearDriveRadiusMeters
            - rotVelRps / DifferentialArm.kSimDifferentialArmRadiusMeters;
    double rightRadPerSec =
        extVelMps / DifferentialArm.kSimLinearDriveRadiusMeters
            + rotVelRps / DifferentialArm.kSimDifferentialArmRadiusMeters;

    // Encoder conversion factors from real hardware
    leftPos = Units.radiansToRotations(leftRad) * DifferentialArm.kEncoderPositionFactor;
    rightPos = Units.radiansToRotations(rightRad) * DifferentialArm.kEncoderPositionFactor;
    leftVel =
        Units.radiansPerSecondToRotationsPerMinute(leftRadPerSec)
            * DifferentialArm.kEncoderVelocityFactor;
    rightVel =
        Units.radiansPerSecondToRotationsPerMinute(rightRadPerSec)
            * DifferentialArm.kEncoderVelocityFactor;

    leftCurrent = sim.getLeftCurrentAmps();
    rightCurrent = sim.getRightCurrentAmps();
  }
}
