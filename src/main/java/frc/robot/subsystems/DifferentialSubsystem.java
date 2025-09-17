package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DifferentialArm;
import frc.robot.commands.Waits.ExtensionAndRotationWait;
import frc.robot.commands.Waits.ExtensionSetWait;
import frc.robot.commands.Waits.RotationSetWait;
import frc.robot.io.DifferentialArmIO;
import frc.robot.io.DifferentialArmIOSim;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;
import frc.utils.LinearInterpolator;

/** Subsystem controlling the differential arm through an IO abstraction. */
public class DifferentialSubsystem extends SubsystemBase implements AutoCloseable {
  private final DifferentialArmIO io;

  private final ProfiledPIDController extensionPid =
      new ProfiledPIDController(30, 0, 1.5, new Constraints(3000, 12000));
  private final ProfiledPIDController rotationPid =
      new ProfiledPIDController(60, 0, 4, new Constraints(1400, 5600));

  private final FlytLogger differentialDash = new FlytLogger("Differential");

  private final double differentialArmRadiusMeters;
  private final double linearDriveRadiusMeters;
  private final double diffToLinearRadiusRatio;
  private final double linearToDiffRadiusRatio;

  private double extensionSetpoint = 0.0;
  private double rotationSetpoint = 0.0;

  private final LinearInterpolator l4RotationInterpolator =
      new LinearInterpolator(DifferentialArm.l4RotationData);
  private final LinearInterpolator l4ExtensionInterpolator =
      new LinearInterpolator(DifferentialArm.l4ExtensionData);
  private final LinearInterpolator l2_3RotationInterpolator =
      new LinearInterpolator(DifferentialArm.l2_3RotationData);
  private final LinearInterpolator l2_3ExtensionInterpolator =
      new LinearInterpolator(DifferentialArm.l2_3ExtensionData);

  private double extensionVelocity;
  private double rotationVelocity;
  private double leftCommand;
  private double rightCommand;

  private int laserCanDistance;
  private boolean hasLaserCanDistance;

  public DifferentialSubsystem(DifferentialArmIO io) {
    this.io = io;
    if (io instanceof DifferentialArmIOSim) {
      linearDriveRadiusMeters = DifferentialArm.kSimLinearDriveRadiusMeters;
      differentialArmRadiusMeters = DifferentialArm.kSimDifferentialArmRadiusMeters;
    } else {
      linearDriveRadiusMeters = DifferentialArm.kLinearDriveRadiusMeters;
      differentialArmRadiusMeters = DifferentialArm.kDifferentialArmRadiusMeters;
    }
    diffToLinearRadiusRatio = differentialArmRadiusMeters / linearDriveRadiusMeters;
    linearToDiffRadiusRatio = linearDriveRadiusMeters / differentialArmRadiusMeters;
    extensionPid.reset(extensionSetpoint);
    rotationPid.reset(rotationSetpoint);

    differentialDash.addDoublePublisher("Left POS", true, this::getLeftPos);
    differentialDash.addDoublePublisher("Right POS", true, this::getRightPos);
    differentialDash.addDoublePublisher("Extension POS", false, this::getExtensionPosition);
    differentialDash.addDoublePublisher("Rotation POS", false, this::getRotationPosition);
    differentialDash.addBoolPublisher("At Extension", false, this::atExtenstionSetpoint);
    differentialDash.addBoolPublisher("At Rotation", false, this::atRotationSetpoint);
    differentialDash.addDoublePublisher("Ext Setpoint", false, this::getExtensionSetpoint);
    differentialDash.addDoublePublisher("Rot Setpoint", false, this::getRotationSetpoint);
    differentialDash.addDoublePublisher("Ext Command Vel", true, () -> extensionVelocity);
    differentialDash.addDoublePublisher(
        "Ext Vel", true, () -> (io.getLeftVelocity() + io.getRightVelocity()) / 2);
    differentialDash.addDoublePublisher("Rot Command Vel", true, () -> rotationVelocity);
    differentialDash.addDoublePublisher(
        "Rot Vel", true, () -> (io.getLeftVelocity() - io.getRightVelocity()) / 2);
    differentialDash.addDoublePublisher("Left Command", true, () -> leftCommand);
    differentialDash.addDoublePublisher("Right Command", true, () -> rightCommand);
    differentialDash.addDoublePublisher("Left Current", true, io::getLeftCurrentAmps);
    differentialDash.addDoublePublisher("Right Current", true, io::getRightCurrentAmps);
    differentialDash.addIntegerPublisher("LaserCan Distance", true, () -> laserCanDistance);
  }

  /** Default constructor using a simple simulation backend. */
  public DifferentialSubsystem() {
    this(new DifferentialArmIOSim());
  }

  public void extend(double setpoint) {
    io.setArmVelocitySetpoints(setpoint, setpoint);
  }

  public void rotate(double setpoint) {
    io.setArmVelocitySetpoints(setpoint, -setpoint);
  }

  public void setExtensionSetpoint(double setpoint) {
    extensionSetpoint = setpoint;
  }

  public Command setExtensionSetpointCommand(double setpoint) {
    return new ExtensionSetWait(this, setpoint);
  }

  public void setRotationSetpoint(double setpoint) {
    rotationSetpoint = setpoint;
  }

  public Command setRotationSetpointCommand(double setpoint) {
    return new RotationSetWait(this, setpoint);
  }

  public Command setRotAndExtSetpointCommand(double ext, double rot) {
    return new ExtensionAndRotationWait(this, ext, rot);
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
    return io.getLeftPosition();
  }

  public double getRightPos() {
    return io.getRightPosition();
  }

  public double getExtensionPosition() {
    return (getLeftPos() + getRightPos()) / 2;
  }

  public double getRotationPosition() {
    double spoolDifferenceMillimeters = (getLeftPos() - getRightPos()) / 2.0;
    double spoolDifferenceMeters = spoolDifferenceMillimeters / 1000.0;
    double rotationRadians = -spoolDifferenceMeters * diffToLinearRadiusRatio;
    return Units.radiansToDegrees(rotationRadians);
  }

  /** Average spool velocity reported by the IO in native units per second. */
  public double getMeasuredExtensionVelocity() {
    return (io.getLeftVelocity() + io.getRightVelocity()) / 2.0;
  }

  /** Differential rotation velocity reported by the IO in native units per second. */
  public double getMeasuredRotationVelocity() {
    return (io.getLeftVelocity() - io.getRightVelocity()) / 2.0;
  }

  public boolean atExtenstionSetpoint() {
    return (extensionSetpoint - 5) <= getExtensionPosition()
        && getExtensionPosition() <= (extensionSetpoint + 5);
  }

  public boolean atRotationSetpoint() {
    return (rotationSetpoint - 5) <= getRotationPosition()
        && getRotationPosition() <= (rotationSetpoint + 5);
  }

  public double getCurrentDraw() {
    return io.getLeftCurrentAmps() + io.getRightCurrentAmps();
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
    double rotationRadians = Units.degreesToRadians(degrees);
    double spoolDifferenceMeters = -rotationRadians * linearToDiffRadiusRatio;
    return spoolDifferenceMeters * 1000.0;
  }

  @Override
  public void periodic() {
    io.update();
    laserCanDistance = io.getLaserDistanceMm();
    hasLaserCanDistance = io.hasLaserDistance();

    extensionVelocity = extensionPid.calculate(getExtensionPosition(), extensionSetpoint);
    rotationVelocity = rotationPid.calculate(getRotationPosition(), rotationSetpoint);
    leftCommand = extensionVelocity - degreesToMM(rotationVelocity);
    rightCommand = extensionVelocity + degreesToMM(rotationVelocity);
    io.setArmVelocitySetpoints(leftCommand, rightCommand);

    differentialDash.update(Constants.debugMode);
  }

  @Override
  public void close() {
    io.close();
  }
}
