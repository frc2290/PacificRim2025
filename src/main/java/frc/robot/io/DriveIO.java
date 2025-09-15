package frc.robot.io;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/** Hardware-agnostic interface for drivetrain inputs and outputs. */
public interface DriveIO {
  /** Command the drivetrain with chassis speeds in meters per second and radians per second. */
  default void setChassisSpeeds(ChassisSpeeds speeds) {}

  /** Directly set desired module states. */
  default void setModuleStates(SwerveModuleState[] states) {}

  /** Zero the gyro heading. */
  default void zeroGyro() {}

  /** Adjust the gyro angle offset in degrees. */
  default void setGyroAdjustment(double adjustment) {}

  /** Retrieve the current gyro angle. */
  default Rotation2d getGyroAngle() {
    return new Rotation2d();
  }

  /** Retrieve the current gyro turn rate in degrees per second. */
  default double getGyroRate() {
    return 0.0;
  }

  /** Get module positions for odometry. */
  default SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[0];
  }

  /** Get measured module states. */
  default SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[0];
  }

  /** Get per-module currents in amps. */
  default double[] getModuleCurrents() {
    return new double[0];
  }

  /** Get applied drive motor outputs. */
  default double[] getDriveAppliedOutputs() {
    return new double[0];
  }

  /** Get the current battery voltage. */
  default double getBatteryVoltage() {
    return 0.0;
  }

  /** Optional field visualization for simulation. */
  default Field2d getField() {
    return null;
  }

  /** Ground-truth pose from the simulator, if available. */
  default Pose2d getSimPose() {
    return new Pose2d();
  }

  /** Underlying drivetrain simulation for MapleSim registration. */
  default SwerveDriveSimulation getDriveSimulation() {
    return null;
  }

  /** Called once per loop for hardware-specific periodic actions. */
  default void periodic() {}
}
