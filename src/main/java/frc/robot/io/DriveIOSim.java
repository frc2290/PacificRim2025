package frc.robot.io;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.DriveConstants;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

/** MapleSim drivetrain implementation for simulation. */
public class DriveIOSim implements DriveIO {
  private final SwerveDriveSimulation driveSim;
  private final SelfControlledSwerveDriveSimulation mapleSim;
  private final SimDeviceSim navxSim;
  private final SimDouble navxYaw;
  private final Field2d field = new Field2d();
  private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();
  private Rotation2d gyroOffset = new Rotation2d();

  public DriveIOSim() {
    driveSim = new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d());
    mapleSim = new SelfControlledSwerveDriveSimulation(driveSim);
    navxSim = new SimDeviceSim("navX-Sensor[0]");
    navxYaw = navxSim.getDouble("Yaw");
  }

  @Override
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    lastChassisSpeeds = speeds;
  }

  @Override
  public void setModuleStates(SwerveModuleState[] states) {
    lastChassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(states);
  }

  @Override
  public void periodic() {
    mapleSim.runChassisSpeeds(lastChassisSpeeds, new Translation2d(), false, true);
    mapleSim.periodic();
    if (navxYaw != null) {
      navxYaw.set(mapleSim.getRawGyroAngle().getDegrees());
    }
  }

  @Override
  public Rotation2d getGyroAngle() {
    return mapleSim.getRawGyroAngle().minus(gyroOffset);
  }

  @Override
  public double getGyroRate() {
    return 0.0; // Not modeled
  }

  @Override
  public void zeroGyro() {
    gyroOffset = mapleSim.getRawGyroAngle();
  }

  @Override
  public void setGyroAdjustment(double adjustment) {
    // No-op for simulation
  }

  @Override
  public SwerveModulePosition[] getModulePositions() {
    return mapleSim.getLatestModulePositions();
  }

  @Override
  public SwerveModuleState[] getModuleStates() {
    return mapleSim.getMeasuredStates();
  }

  @Override
  public double[] getModuleCurrents() {
    var modules = driveSim.getModules();
    double[] currents = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currents[i] =
          modules[i].getDriveMotorSupplyCurrent().in(edu.wpi.first.units.Units.Amps)
              + modules[i].getSteerMotorSupplyCurrent().in(edu.wpi.first.units.Units.Amps);
    }
    return currents;
  }

  @Override
  public double[] getDriveAppliedOutputs() {
    return new double[0];
  }

  @Override
  public double getBatteryVoltage() {
    return SimulatedBattery.getBatteryVoltage().in(edu.wpi.first.units.Units.Volts);
  }

  @Override
  public Field2d getField() {
    return field;
  }

  @Override
  public Pose2d getSimPose() {
    return driveSim.getSimulatedDriveTrainPose();
  }

  @Override
  public SwerveDriveSimulation getDriveSimulation() {
    return driveSim;
  }
}
