package frc.robot.io;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.MAXSwerveModule;
import java.util.Arrays;

/** Real-hardware drivetrain implementation. */
public class DriveIOReal implements DriveIO {
  private final MAXSwerveModule frontLeft;
  private final MAXSwerveModule frontRight;
  private final MAXSwerveModule rearLeft;
  private final MAXSwerveModule rearRight;
  private final MAXSwerveModule[] modules;
  private final AHRS gyro;

  public DriveIOReal() {
    frontLeft =
        new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);
    frontRight =
        new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);
    rearLeft =
        new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);
    rearRight =
        new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);
    modules = new MAXSwerveModule[] {frontLeft, frontRight, rearLeft, rearRight};
    gyro = new AHRS(NavXComType.kMXP_SPI);
    gyro.setAngleAdjustment(180);
  }

  @Override
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    var states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(states);
  }

  @Override
  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  @Override
  public Rotation2d getGyroAngle() {
    return gyro.getRotation2d();
  }

  @Override
  public double getGyroRate() {
    return gyro.getRate();
  }

  @Override
  public void zeroGyro() {
    gyro.reset();
  }

  @Override
  public void setGyroAdjustment(double adjustment) {
    gyro.setAngleAdjustment(adjustment);
  }

  @Override
  public SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(modules)
        .map(MAXSwerveModule::getPosition)
        .toArray(SwerveModulePosition[]::new);
  }

  @Override
  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules).map(MAXSwerveModule::getState).toArray(SwerveModuleState[]::new);
  }

  @Override
  public double[] getModuleCurrents() {
    return Arrays.stream(modules).mapToDouble(MAXSwerveModule::getCurrentDraw).toArray();
  }

  @Override
  public double[] getDriveAppliedOutputs() {
    return new double[] {
      frontLeft.getDriveAppliedOutput(),
      frontRight.getDriveAppliedOutput(),
      rearLeft.getDriveAppliedOutput(),
      rearRight.getDriveAppliedOutput()
    };
  }

  @Override
  public double getBatteryVoltage() {
    return RobotController.getBatteryVoltage();
  }

  // Real hardware has no simulation field or pose
  @Override
  public Pose2d getSimPose() {
    return new Pose2d();
  }
}
