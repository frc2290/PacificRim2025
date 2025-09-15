package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.io.DriveIO;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/** Drivetrain subsystem backed by a pluggable IO layer. */
public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  private final DriveIO io;

  // Controllers
  private double slowSpeed = 1.0;
  private final PIDController rotPid = new PIDController(0.01, 0.0, 0.0);
  private final PIDController xPid = new PIDController(1, 0.0, 0.085);
  private final PIDController yPid = new PIDController(1, 0.0, 0.085);

  // Odometry
  private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          new Rotation2d(),
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          });

  // NetworkTables publishers
  private final StructArrayPublisher<SwerveModuleState> moduleStatePublisher;
  private final DoublePublisher cmdVxPublisher;
  private final DoublePublisher cmdVyPublisher;
  private final DoublePublisher cmdOmegaPublisher;
  private final DoublePublisher actVxPublisher;
  private final DoublePublisher actVyPublisher;
  private final DoublePublisher actOmegaPublisher;
  private final DoubleArrayPublisher moduleCurrentPublisher;
  private final DoubleArrayPublisher moduleOutputPublisher;
  private final DoublePublisher batteryVoltagePublisher;

  // Last commanded speeds
  private ChassisSpeeds lastCommandedSpeeds = new ChassisSpeeds();

  // SysId routines
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(null, Volts.of(4), Seconds.of(5)),
          new SysIdRoutine.Mechanism(
              (voltage) -> this.runDriveCharacterization(voltage.in(Volts)), null, this));

  private final SysIdRoutine sysIdRoutineTurn =
      new SysIdRoutine(
          new SysIdRoutine.Config(null, Volts.of(4), Seconds.of(5)),
          new SysIdRoutine.Mechanism(
              (voltage) -> this.runTurnCharacterization(voltage.in(Volts)), null, this));

  public DriveSubsystem(DriveIO io) {
    this.io = io;
    rotPid.enableContinuousInput(0, 360);

    var nt = NetworkTableInstance.getDefault();
    moduleStatePublisher =
        nt.getStructArrayTopic("Drive/ModuleStates", SwerveModuleState.struct).publish();
    cmdVxPublisher = nt.getDoubleTopic("Drive/Commanded/Vx").publish();
    cmdVyPublisher = nt.getDoubleTopic("Drive/Commanded/Vy").publish();
    cmdOmegaPublisher = nt.getDoubleTopic("Drive/Commanded/Omega").publish();
    actVxPublisher = nt.getDoubleTopic("Drive/Actual/Vx").publish();
    actVyPublisher = nt.getDoubleTopic("Drive/Actual/Vy").publish();
    actOmegaPublisher = nt.getDoubleTopic("Drive/Actual/Omega").publish();
    moduleCurrentPublisher = nt.getDoubleArrayTopic("Drive/ModuleCurrents").publish();
    moduleOutputPublisher = nt.getDoubleArrayTopic("Drive/AppliedOutputs").publish();
    batteryVoltagePublisher = nt.getDoubleTopic("Drive/BatteryVoltage").publish();

    SmartDashboard.putData("Drive Rot PID", rotPid);
    SmartDashboard.putData("Drive X PID", xPid);
    SmartDashboard.putData("Drive Y PID", yPid);
  }

  @Override
  public void periodic() {
    io.periodic();

    odometry.update(io.getGyroAngle(), io.getModulePositions());
    moduleStatePublisher.set(io.getModuleStates());

    var actual = DriveConstants.kDriveKinematics.toChassisSpeeds(io.getModuleStates());
    actVxPublisher.set(actual.vxMetersPerSecond);
    actVyPublisher.set(actual.vyMetersPerSecond);
    actOmegaPublisher.set(actual.omegaRadiansPerSecond);
    moduleCurrentPublisher.set(io.getModuleCurrents());
    moduleOutputPublisher.set(io.getDriveAppliedOutputs());
    batteryVoltagePublisher.set(io.getBatteryVoltage());

    Field2d field = io.getField();
    if (field != null) {
      field.setRobotPose(io.getSimPose());
    }
  }

  // Basic accessors
  public Field2d getField() {
    return io.getField();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Pose2d getSimPose() {
    return io.getSimPose();
  }

  public SwerveDriveSimulation getDriveSimulation() {
    return io.getDriveSimulation();
  }

  public double[] getModuleCurrents() {
    return io.getModuleCurrents();
  }

  public double[] getDriveAppliedOutputs() {
    return io.getDriveAppliedOutputs();
  }

  public SwerveModulePosition[] getModulePositions() {
    return io.getModulePositions();
  }

  public SwerveModuleState[] getModuleStates() {
    return io.getModuleStates();
  }

  /** Returns the total current draw of all modules. */
  public double getCurrentDraw() {
    double total = 0.0;
    for (double c : io.getModuleCurrents()) {
      total += c;
    }
    return total;
  }

  public double getHeading() {
    return io.getGyroAngle().getDegrees();
  }

  public Rotation2d newHeading() {
    return io.getGyroAngle();
  }

  public void zeroHeading() {
    io.zeroGyro();
  }

  public void setGyroAdjustment(double adjustment) {
    io.setGyroAdjustment(adjustment);
  }

  // Driving methods
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond * slowSpeed;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond * slowSpeed;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed * slowSpeed;

    ChassisSpeeds commanded =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                Rotation2d.fromDegrees(-getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    lastCommandedSpeeds = commanded;
    cmdVxPublisher.set(commanded.vxMetersPerSecond);
    cmdVyPublisher.set(commanded.vyMetersPerSecond);
    cmdOmegaPublisher.set(commanded.omegaRadiansPerSecond);
    io.setChassisSpeeds(commanded);
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    lastCommandedSpeeds = speeds;
    cmdVxPublisher.set(speeds.vxMetersPerSecond);
    cmdVyPublisher.set(speeds.vyMetersPerSecond);
    cmdOmegaPublisher.set(speeds.omegaRadiansPerSecond);
    io.setChassisSpeeds(speeds);
  }

  public void setX() {
    io.setModuleStates(
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        });
  }

  // Utilities
  public void setSlowSpeed() {
    slowSpeed = 0.5;
  }

  public void setRegularSpeed() {
    slowSpeed = 1.0;
  }

  public boolean isSlowSpeed() {
    return slowSpeed < 1.0;
  }

  public PIDController getRotPidController() {
    return rotPid;
  }

  public PIDController getXPidController() {
    return xPid;
  }

  public PIDController getYPidController() {
    return yPid;
  }

  public Command getQuasistaticForward(boolean turn) {
    return turn
        ? sysIdRoutineTurn.quasistatic(SysIdRoutine.Direction.kForward)
        : sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command getQuasistaticReverse(boolean turn) {
    return turn
        ? sysIdRoutineTurn.quasistatic(SysIdRoutine.Direction.kReverse)
        : sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command getDynamicForward(boolean turn) {
    return turn
        ? sysIdRoutineTurn.dynamic(SysIdRoutine.Direction.kForward)
        : sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command getDynamicReverse(boolean turn) {
    return turn
        ? sysIdRoutineTurn.dynamic(SysIdRoutine.Direction.kReverse)
        : sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
  }

  @Override
  public void close() {}

  private void runDriveCharacterization(double output) {
    // No-op in IO abstraction; handled by IO implementations if needed
  }

  private void runTurnCharacterization(double output) {
    // No-op in IO abstraction; handled by IO implementations if needed
  }
}
