// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DriveSubsystem extends SubsystemBase {

    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset);

    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset);

    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kBackLeftChassisAngularOffset);

    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kBackRightChassisAngularOffset);

    // The gyro sensor
    private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
    // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

    private double slowSpeed = 1.0;

    private PIDController rotPid = new PIDController(0.01, 0.0, 0.0); // 0.015 0 0
    private PIDController xPid = new PIDController(1, 0.0, 0.085); // 2 0.0 0.5
    private PIDController yPid = new PIDController(1, 0.0, 0.085); // 2 0.0 0.5

    // Create the SysId routine
    private SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), Seconds.of(5)),
        new SysIdRoutine.Mechanism(
            (voltage) -> this.runDriveCharacterization(voltage.in(Volts)),
            null, // No log consumer, since data is recorded by URCL
            this
        )
    );

    private SysIdRoutine sysIdRoutineTurn = new SysIdRoutine(
        new SysIdRoutine.Config(null, Volts.of(4), Seconds.of(5)),
        new SysIdRoutine.Mechanism(
            (voltage) -> this.runTurnCharacterization(voltage.in(Volts)),
            null, // No log consumer, since data is recorded by URCL
            this
        )
    );

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // Usage reporting for MAXSwerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
        m_gyro.setAngleAdjustment(180);
        rotPid.enableContinuousInput(0, 360);

        SmartDashboard.putData("Drive Rot PID", rotPid);
        SmartDashboard.putData("Drive X PID", xPid);
        SmartDashboard.putData("Drive Y PID", yPid);
    }

    public void setGyroAdjustment(double adjustment) {
        m_gyro.setAngleAdjustment(adjustment);
    }

    @Override
    public void periodic() {
        // Nothing
    }

    public void setSlowSpeed() {
        slowSpeed = 0.5;
    }

    public void setRegularSpeed() {
        slowSpeed = 1;
    }

    public boolean isSlowSpeed() {
        return slowSpeed < 1;
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
        return turn ? sysIdRoutineTurn.quasistatic(SysIdRoutine.Direction.kForward) : sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    }

    public Command getQuasistaticReverse(boolean turn) {
        return turn ? sysIdRoutineTurn.quasistatic(SysIdRoutine.Direction.kReverse) : sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
    }

    public Command getDynamicForward(boolean turn) {
        return turn ? sysIdRoutineTurn.dynamic(SysIdRoutine.Direction.kForward) : sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
    }

    public Command getDynamicReverse(boolean turn) {
        return turn ? sysIdRoutineTurn.dynamic(SysIdRoutine.Direction.kReverse) : sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        // double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond * slowSpeed;
        // double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond * slowSpeed;
        // double rotDelivered = rot * DriveConstants.kMaxAngularSpeed * slowSpeed;
        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                Rotation2d.fromDegrees(-m_gyro.getAngle()))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    public void driveChassisSpeeds(ChassisSpeeds speeds) {
        // System.out.println(speeds.vxMetersPerSecond + " " + speeds.vyMetersPerSecond
        // + " " + speeds.omegaRadiansPerSecond);
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[] {
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState()
        };
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
        };
        return positions;
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        m_gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        // return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
        return (m_gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public Rotation2d newHeading() {
        return m_gyro.getRotation2d();
        //return Rotation2d.fromDegrees(-m_gyro.getAngle());
    }

    public void setDriveCoast() {
        m_frontLeft.setDriveCoast();
        m_frontRight.setDriveCoast();
        m_rearLeft.setDriveCoast();
        m_rearRight.setDriveCoast();
    }

    private void runDriveCharacterization(double output) {
        m_frontLeft.runDriveCharacterization(output);
        m_frontRight.runDriveCharacterization(output);
        m_rearLeft.runDriveCharacterization(output);
        m_rearRight.runDriveCharacterization(output);
    }

    private void runTurnCharacterization(double output) {
        m_frontLeft.runTurnCharacterization(output);
        m_frontRight.runTurnCharacterization(output);
        m_rearLeft.runTurnCharacterization(output);
        m_rearRight.runTurnCharacterization(output);
    }
}