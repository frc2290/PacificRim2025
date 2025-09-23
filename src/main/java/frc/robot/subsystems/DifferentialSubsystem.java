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

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DifferentialArm;
import frc.robot.commands.Waits.ExtensionAndRotationWait;
import frc.robot.commands.Waits.ExtensionSetWait;
import frc.robot.commands.Waits.RotationSetWait;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;
import frc.utils.LinearInterpolator;

/** Owns the differential arm that controls manipulator rotation and extension. */
public class DifferentialSubsystem extends SubsystemBase {

  /** Motion-profiled PID that handles telescoping the arm in millimeters. */
  private ProfiledPIDController extensionPid =
      new ProfiledPIDController(
          DifferentialArm.kExtensionProfiledKp,
          DifferentialArm.kExtensionProfiledKi,
          DifferentialArm.kExtensionProfiledKd,
          new Constraints(
              DifferentialArm.kExtensionMaxVelocityMillimetersPerSecond,
              DifferentialArm.kExtensionMaxAccelerationMillimetersPerSecondSquared));

  /** Motion-profiled PID that rotates the arm while respecting acceleration limits. */
  private ProfiledPIDController rotationPid =
      new ProfiledPIDController(
          DifferentialArm.kRotationProfiledKp,
          DifferentialArm.kRotationProfiledKi,
          DifferentialArm.kRotationProfiledKd,
          new Constraints(
              DifferentialArm.kRotationMaxVelocityDegreesPerSecond,
              DifferentialArm.kRotationMaxAccelerationDegreesPerSecondSquared));

  private SparkMax leftMotor;
  private SparkMax rightMotor;

  private SparkMaxConfig leftConfig = new SparkMaxConfig();
  private SparkMaxConfig rightConfig = new SparkMaxConfig();

  private RelativeEncoder leftEnc;
  private RelativeEncoder rightEnc;

  private SparkClosedLoopController leftArm;
  private SparkClosedLoopController rightArm;

  /** Dashboard helper for streaming arm telemetry to AdvantageScope. */
  private FlytLogger differentialDash = new FlytLogger("Differential");

  private double extensionSetpoint = 0;
  private double rotationSetpoint = 0;

  @SuppressWarnings("unused")
  private SlewRateLimiter extendSlew =
      new SlewRateLimiter(DifferentialArm.kExtensionSlewRateMillimetersPerSecond);

  @SuppressWarnings("unused")
  private SlewRateLimiter rotateSlew =
      new SlewRateLimiter(DifferentialArm.kRotationSlewRateDegreesPerSecond);

  /** Laser rangefinder mounted near the manipulator for interpolation of scoring positions. */
  private LaserCan lc;

  private int laserCanDistance = 0;
  private boolean hasLaserCanDistance = false;

  /** Lookup table for upper reef rotations keyed by laser distance. */
  private LinearInterpolator l4RotationInterpolator =
      new LinearInterpolator(DifferentialArm.l4RotationData);

  /** Lookup table for upper reef extensions keyed by laser distance. */
  private LinearInterpolator l4ExtensionInterpolator =
      new LinearInterpolator(DifferentialArm.l4ExtensionData);

  /** Lookup table for mid reef rotations keyed by laser distance. */
  private LinearInterpolator l2_3RotationInterpolator =
      new LinearInterpolator(DifferentialArm.l2_3RotationData);

  /** Lookup table for mid reef extensions keyed by laser distance. */
  private LinearInterpolator l2_3ExtensionInterpolator =
      new LinearInterpolator(DifferentialArm.l2_3ExtensionData);

  double extensionVelocity;
  double rotationVelocity;
  double leftCommand;
  double rightCommand;

  public DifferentialSubsystem() {
    leftMotor = new SparkMax(DifferentialArm.kLeftMotorId, MotorType.kBrushless);
    rightMotor = new SparkMax(DifferentialArm.kRightMotorId, MotorType.kBrushless);

    leftConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DifferentialArm.kArmCurrentLimit)
        .encoder
        .positionConversionFactor(DifferentialArm.kPositionConversionFactor)
        .velocityConversionFactor(DifferentialArm.kVelocityConversionFactor)
        .quadratureMeasurementPeriod(DifferentialArm.kQuadratureMeasurementPeriod)
        .quadratureAverageDepth(DifferentialArm.kQuadratureAverageDepth);
    leftConfig
        .closedLoop
        .p(DifferentialArm.kVelocityLoopP, ClosedLoopSlot.kSlot0)
        .i(DifferentialArm.kVelocityLoopI, ClosedLoopSlot.kSlot0)
        .d(DifferentialArm.kVelocityLoopD, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1);

    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightConfig.apply(leftConfig);

    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftEnc = leftMotor.getEncoder();
    leftEnc.setPosition(0);
    rightEnc = rightMotor.getEncoder();
    rightEnc.setPosition(0);

    leftArm = leftMotor.getClosedLoopController();
    rightArm = rightMotor.getClosedLoopController();

    extensionPid.reset(extensionSetpoint);
    extensionPid.setTolerance(DifferentialArm.kExtensionPositionToleranceMillimeters);
    rotationPid.reset(rotationSetpoint);
    rotationPid.setTolerance(DifferentialArm.kRotationToleranceDegrees);

    lc = new LaserCan(DifferentialArm.kLaserCanId);

    // Configure the LaserCAN for close range measurements so the interpolators have
    // reliable data.
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(
          new LaserCan.RegionOfInterest(
              DifferentialArm.kLaserRegionX,
              DifferentialArm.kLaserRegionY,
              DifferentialArm.kLaserRegionWidth,
              DifferentialArm.kLaserRegionHeight));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("LaserCan Configuration failed! " + e);
    }

    differentialDash.addDoublePublisher("Left POS", true, () -> getLeftPos());
    differentialDash.addDoublePublisher("Right POS", true, () -> getRightPos());
    differentialDash.addDoublePublisher("Extension POS", false, () -> getExtensionPosition());
    differentialDash.addDoublePublisher("Rotation POS", false, () -> getRotationPosition());
    differentialDash.addBoolPublisher("At Extension", false, () -> atExtenstionSetpoint());
    differentialDash.addBoolPublisher("At Rotation", false, () -> atRotationSetpoint());
    differentialDash.addDoublePublisher("Ext Setpoint", false, () -> getExtensionSetpoint());
    differentialDash.addDoublePublisher("Rot Setpoint", false, () -> getRotationSetpoint());
    differentialDash.addDoublePublisher("Left Output", true, () -> leftMotor.getAppliedOutput());
    differentialDash.addDoublePublisher("Left Current", true, () -> leftMotor.getOutputCurrent());
    differentialDash.addDoublePublisher("Right Output", true, () -> rightMotor.getAppliedOutput());
    differentialDash.addDoublePublisher("Right Current", true, () -> rightMotor.getOutputCurrent());
    differentialDash.addDoublePublisher("Voltage", true, () -> leftMotor.getBusVoltage());
    differentialDash.addDoublePublisher("Ext Command Vel", true, () -> extensionVelocity);
    differentialDash.addDoublePublisher(
        "Ext Vel", true, () -> (leftEnc.getVelocity() + rightEnc.getVelocity()) / 2);
    differentialDash.addDoublePublisher("Rot Command Vel", true, () -> rotationVelocity);
    differentialDash.addDoublePublisher(
        "Rot Vel", true, () -> (leftEnc.getVelocity() - rightEnc.getVelocity()) / 2);
    differentialDash.addDoublePublisher("Left Command", true, () -> leftCommand);
    differentialDash.addDoublePublisher("Right Command", true, () -> rightCommand);
    differentialDash.addIntegerPublisher("LaserCan Distance", true, () -> laserCanDistance);
  }

  /** Direct percent-output control for telescoping, primarily used for testing. */
  public void extend(double setpoint) {
    leftMotor.set(setpoint);
    rightMotor.set(setpoint);
  }

  /** Direct percent-output control for rotation, primarily used for testing. */
  public void rotate(double setpoint) {
    leftMotor.set(setpoint);
    rightMotor.set(-setpoint);
  }

  public void setExtensionSetpoint(double setpoint) {
    extensionSetpoint = setpoint;
  }

  public Command setExtensionSetpointCommand(double setpoint) {
    return new ExtensionSetWait(this, setpoint);
    // return Commands.run(() -> setExtensionSetpoint(setpoint)).until(() ->
    // atExtenstionSetpoint());
  }

  public void setRotationSetpoint(double setpoint) {
    rotationSetpoint = setpoint;
  }

  public Command setRotationSetpointCommand(double setpoint) {
    return new RotationSetWait(this, setpoint);
    // return Commands.run(() -> setRotationSetpoint(setpoint)).until(() ->
    // atRotationSetpoint());
  }

  public Command setRotAndExtSetpointCommand(double ext, double rot) {
    return new ExtensionAndRotationWait(this, ext, rot);
    // return Commands.run(() -> {
    // setExtensionSetpoint(ext);
    // setRotationSetpoint(rot);
    // }).until(() -> {
    // return atRotationSetpoint() && atExtenstionSetpoint();
    // });
  }

  public double getExtensionSetpoint() {
    return extensionSetpoint;
  }

  public double getRotationSetpoint() {
    return rotationSetpoint;
  }

  public Command incrementExtensionSetpoint(double increment) {
    return Commands.runOnce(() -> extensionSetpoint += increment);
  }

  public Command incrementRotationSetpoint(double increment) {
    return Commands.runOnce(() -> rotationSetpoint += increment);
  }

  public double getLeftPos() {
    return leftEnc.getPosition();
  }

  public double getRightPos() {
    return rightEnc.getPosition();
  }

  public double getExtensionPosition() {
    return (getLeftPos() + getRightPos()) / 2;
    // return (motor1.getPos()+motor2.getPos())/2;
  }

  public double getRotationPosition() {
    return -(((getLeftPos() - getRightPos()) / 2) / DifferentialArm.kMillimetersPerRotation)
        * DifferentialArm.kDegreesPerRotation;
    // return endeffector.getWristPos();
  }

  public boolean atExtenstionSetpoint() {
    return extensionPid.atSetpoint() && isExtenstionProfileFinished();
  }

  public boolean atRotationSetpoint() {
    return rotationPid.atSetpoint() && isRotationProfileFinished();
  }

  private boolean isExtenstionProfileFinished() {
    var goal = extensionPid.getGoal();
    var setpoint = extensionPid.getSetpoint();
    return MathUtil.isNear(
        goal.position,
        setpoint.position,
        DifferentialArm.kExtensionProfileGoalPositionTolerance);
  }

  private boolean isRotationProfileFinished() {
    var goal = rotationPid.getGoal();
    var setpoint = rotationPid.getSetpoint();
    return MathUtil.isNear(
        goal.position,
        setpoint.position,
        DifferentialArm.kRotationProfileGoalPositionTolerance);
  }

  public int getLaserCanDistance() {
    return laserCanDistance;
  }

  public boolean hasLaserCanDistance() {
    return hasLaserCanDistance;
  }

  public double l4RotationInterpolate() {
    return l4RotationInterpolator.getInterpolatedValue(laserCanDistance);
  }

  public double l4ExtensionInterpolate() {
    return l4ExtensionInterpolator.getInterpolatedValue(laserCanDistance);
  }

  public double l2_3RotationInterpolate() {
    return l2_3RotationInterpolator.getInterpolatedValue(laserCanDistance);
  }

  public double l2_3ExtensionInterpolate() {
    return l2_3ExtensionInterpolator.getInterpolatedValue(laserCanDistance);
  }

  private double degreesToMM(double degrees) {
    return (degrees / DifferentialArm.kDegreesPerRotation)
        * DifferentialArm.kMillimetersPerRotation;
  }

  @Override
  public void periodic() {
    // Poll the LaserCAN for distance data that helps compute interpolated presets.
    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null
        && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        && measurement.distance_mm < DifferentialArm.kLaserMaxValidDistanceMillimeters) {
      laserCanDistance = measurement.distance_mm;
      hasLaserCanDistance = true;
      // System.out.println("The target is " + measurement.distance_mm + "mm away!");
    } else {
      hasLaserCanDistance = false;
    }

    // Convert PID outputs into velocity commands for each motor. Extension is the
    // average, rotation
    // is produced by commanding opposite directions on the two motors.
    extensionVelocity = extensionPid.calculate(getExtensionPosition(), extensionSetpoint);
    rotationVelocity = rotationPid.calculate(getRotationPosition(), rotationSetpoint);
    // double extFeed = extFeedforward.calculate(extensionVelocity);
    // double rotFeed =
    // rotFeedforward.calculate((degreesToRadians(getRotationPosition()) - 0.139),
    // rotationVelocity);
    leftCommand = extensionVelocity - degreesToMM(rotationVelocity);
    rightCommand = extensionVelocity + degreesToMM(rotationVelocity);
    leftArm.setReference(extensionVelocity - degreesToMM(rotationVelocity), ControlType.kVelocity);
    rightArm.setReference(extensionVelocity + degreesToMM(rotationVelocity), ControlType.kVelocity);
    // leftArm.setReference(extendSlew.calculate(extensionSetpoint) -
    // degreesToMM(rotateSlew.calculate(rotationSetpoint)), ControlType.kPosition);
    // rightArm.setReference(extendSlew.calculate(extensionSetpoint) +
    // degreesToMM(rotateSlew.calculate(rotationSetpoint)), ControlType.kPosition);
    differentialDash.update(Constants.debugMode);
  }
}
