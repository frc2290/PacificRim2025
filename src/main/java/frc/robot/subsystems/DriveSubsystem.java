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
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.utils.SwerveDriveSim;
import frc.utils.SwerveModuleSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DriveSubsystem extends SubsystemBase {

    // Create MAXSwerveModules
    private final MAXSwerveModule m_frontLeft;
    private final MAXSwerveModule m_frontRight;
    private final MAXSwerveModule m_rearLeft;
    private final MAXSwerveModule m_rearRight;

    private final MAXSwerveModule[] m_modules;

    // The gyro sensor
    private final AHRS m_gyro;
    // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

    private double slowSpeed = 1.0;

    private PIDController rotPid = new PIDController(0.01, 0.0, 0.0); // 0.015 0 0
    private PIDController xPid = new PIDController(1, 0.0, 0.085); // 2 0.0 0.5
    private PIDController yPid = new PIDController(1, 0.0, 0.085); // 2 0.0 0.5

    // Simulation state
    private Field2d m_field;
    private final SimDeviceSim navxSim = RobotBase.isSimulation() ? new SimDeviceSim("navX-Sensor[0]") : null;
    private final SimDouble navxYaw = navxSim != null ? navxSim.getDouble("Yaw") : null;
    private final SwerveDriveSim swerveDriveSim;
    private final SwerveModuleSim[] m_moduleSims;
    private final SwerveDriveOdometry m_odometry;

    // AdvantageScope publishers
    private final StructArrayPublisher<SwerveModuleState> moduleStatePublisher;
    private final DoublePublisher cmdVxPublisher;
    private final DoublePublisher cmdVyPublisher;
    private final DoublePublisher cmdOmegaPublisher;
    private final DoublePublisher actVxPublisher;
    private final DoublePublisher actVyPublisher;
    private final DoublePublisher actOmegaPublisher;

    // Store last commanded speeds (robot frame)
    private ChassisSpeeds m_lastCommandedSpeeds = new ChassisSpeeds();

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
        this(
                new MAXSwerveModule(
                        DriveConstants.kFrontLeftDrivingCanId,
                        DriveConstants.kFrontLeftTurningCanId,
                        DriveConstants.kFrontLeftChassisAngularOffset),
                new MAXSwerveModule(
                        DriveConstants.kFrontRightDrivingCanId,
                        DriveConstants.kFrontRightTurningCanId,
                        DriveConstants.kFrontRightChassisAngularOffset),
                new MAXSwerveModule(
                        DriveConstants.kRearLeftDrivingCanId,
                        DriveConstants.kRearLeftTurningCanId,
                        DriveConstants.kBackLeftChassisAngularOffset),
                new MAXSwerveModule(
                        DriveConstants.kRearRightDrivingCanId,
                        DriveConstants.kRearRightTurningCanId,
                        DriveConstants.kBackRightChassisAngularOffset),
                new AHRS(NavXComType.kMXP_SPI));

        // Usage reporting for MAXSwerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    }

    /**
     * Creates a drive subsystem with injected modules and gyro for testing.
     *
     * @param frontLeft  front left swerve module
     * @param frontRight front right swerve module
     * @param rearLeft   rear left swerve module
     * @param rearRight  rear right swerve module
     * @param gyro       gyro sensor
     */
    public DriveSubsystem(MAXSwerveModule frontLeft, MAXSwerveModule frontRight,
            MAXSwerveModule rearLeft, MAXSwerveModule rearRight, AHRS gyro) {
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_rearLeft = rearLeft;
        m_rearRight = rearRight;
        m_modules = new MAXSwerveModule[] { m_frontLeft, m_frontRight, m_rearLeft, m_rearRight };
        m_gyro = gyro;

        m_gyro.setAngleAdjustment(180);
        rotPid.enableContinuousInput(0, 360);

        moduleStatePublisher =
            NetworkTableInstance.getDefault()
                .getStructArrayTopic("Drive/ModuleStates", SwerveModuleState.struct)
                .publish();

        // Chassis speeds publishers (m/s and rad/s) under Drive/Commanded and Drive/Actual
        var nt = NetworkTableInstance.getDefault();
        cmdVxPublisher = nt.getDoubleTopic("Drive/Commanded/Vx").publish();
        cmdVyPublisher = nt.getDoubleTopic("Drive/Commanded/Vy").publish();
        cmdOmegaPublisher = nt.getDoubleTopic("Drive/Commanded/Omega").publish();
        actVxPublisher = nt.getDoubleTopic("Drive/Actual/Vx").publish();
        actVyPublisher = nt.getDoubleTopic("Drive/Actual/Vy").publish();
        actOmegaPublisher = nt.getDoubleTopic("Drive/Actual/Omega").publish();

        SmartDashboard.putData("Drive Rot PID", rotPid);
        SmartDashboard.putData("Drive X PID", xPid);
        SmartDashboard.putData("Drive Y PID", yPid);

        m_odometry = new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            new Rotation2d(),
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            });

        if (RobotBase.isSimulation()) {
            m_field = new Field2d();
            SmartDashboard.putData("Field", m_field);

            m_moduleSims = new SwerveModuleSim[] {
                new SwerveModuleSim(
                    ModuleConstants.kDriveMotor,
                    ModuleConstants.kDrivingMotorReduction,
                    ModuleConstants.kWheelDiameterMeters / 2.0,
                    ModuleConstants.kDriveEfficiency,
                    m_frontLeft.getTurnSim() != null ? ModuleConstants.kSteerMotor : null,
                    ModuleConstants.kSteerReduction,
                    m_frontLeft.getDriveSim(),
                    m_frontLeft.getDrivingController(),
                    m_frontLeft.getTurnSim(),
                    m_frontLeft.getTurningController()),
                new SwerveModuleSim(
                    ModuleConstants.kDriveMotor,
                    ModuleConstants.kDrivingMotorReduction,
                    ModuleConstants.kWheelDiameterMeters / 2.0,
                    ModuleConstants.kDriveEfficiency,
                    m_frontRight.getTurnSim() != null ? ModuleConstants.kSteerMotor : null,
                    ModuleConstants.kSteerReduction,
                    m_frontRight.getDriveSim(),
                    m_frontRight.getDrivingController(),
                    m_frontRight.getTurnSim(),
                    m_frontRight.getTurningController()),
                new SwerveModuleSim(
                    ModuleConstants.kDriveMotor,
                    ModuleConstants.kDrivingMotorReduction,
                    ModuleConstants.kWheelDiameterMeters / 2.0,
                    ModuleConstants.kDriveEfficiency,
                    m_rearLeft.getTurnSim() != null ? ModuleConstants.kSteerMotor : null,
                    ModuleConstants.kSteerReduction,
                    m_rearLeft.getDriveSim(),
                    m_rearLeft.getDrivingController(),
                    m_rearLeft.getTurnSim(),
                    m_rearLeft.getTurningController()),
                new SwerveModuleSim(
                    ModuleConstants.kDriveMotor,
                    ModuleConstants.kDrivingMotorReduction,
                    ModuleConstants.kWheelDiameterMeters / 2.0,
                    ModuleConstants.kDriveEfficiency,
                    m_rearRight.getTurnSim() != null ? ModuleConstants.kSteerMotor : null,
                    ModuleConstants.kSteerReduction,
                    m_rearRight.getDriveSim(),
                    m_rearRight.getDrivingController(),
                    m_rearRight.getTurnSim(),
                    m_rearRight.getTurningController())
            };

            // Align simulated absolute encoders so that modules are "forward"
            // relative to the robot at startup (relative angle = 0 for all).
            // This matches the REV offset convention used by MAXSwerveModule.
            m_moduleSims[0].setEncoderOffset(DriveConstants.kFrontLeftChassisAngularOffset);
            m_moduleSims[1].setEncoderOffset(DriveConstants.kFrontRightChassisAngularOffset);
            m_moduleSims[2].setEncoderOffset(DriveConstants.kBackLeftChassisAngularOffset);
            m_moduleSims[3].setEncoderOffset(DriveConstants.kBackRightChassisAngularOffset);

            swerveDriveSim = new SwerveDriveSim(
                java.util.List.of(m_moduleSims),
                DriveConstants.kModuleTranslations,
                DriveConstants.kRobotMassKg,
                DriveConstants.kRobotMomentOfInertia,
                DriveConstants.kLinearDampingCoeff,
                DriveConstants.kAngularDampingCoeff);
        } else {
            m_field = null;
            m_moduleSims = null;
            swerveDriveSim = null;
        }
    }

    public void setGyroAdjustment(double adjustment) {
        m_gyro.setAngleAdjustment(adjustment);
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            m_odometry.update(m_gyro.getRotation2d(), getModulePositions());
            moduleStatePublisher.set(getModuleStates());

            // Publish actual chassis speeds computed from current module states
            var actual = DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
            actVxPublisher.set(actual.vxMetersPerSecond);
            actVyPublisher.set(actual.vyMetersPerSecond);
            actOmegaPublisher.set(actual.omegaRadiansPerSecond);
        }
    }

    public Field2d getField() {
        return m_field;
    }

    /** Returns the current estimated pose. */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /** Returns the ground-truth pose from the drivetrain simulator. */
    public Pose2d getSimPose() {
        return swerveDriveSim != null ? swerveDriveSim.getPose() : new Pose2d();
    }

    /**
     * Returns the total current draw of all drive modules.
     *
     * @return Sum of currents from each swerve module's drive and turn motors.
     */
    public double getCurrentDraw() {
        double total = 0.0;
        for (MAXSwerveModule module : m_modules) {
            total += module.getCurrentDraw();
        }
        return total;
    }

    @Override
    public void simulationPeriodic() {
        if (swerveDriveSim == null) {
            return;
        }

        double batteryVoltage = RobotController.getBatteryVoltage();
        SwerveModuleSim.ModuleForce[] forces = new SwerveModuleSim.ModuleForce[m_moduleSims.length];
        var speeds = swerveDriveSim.getSpeeds();
        double[] offsets = {
            DriveConstants.kFrontLeftChassisAngularOffset,
            DriveConstants.kFrontRightChassisAngularOffset,
            DriveConstants.kBackLeftChassisAngularOffset,
            DriveConstants.kBackRightChassisAngularOffset
        };
        for (int i = 0; i < m_moduleSims.length; i++) {
            SwerveModuleState desired = m_modules[i].getDesiredState();
            forces[i] = m_moduleSims[i].update(
                desired.speedMetersPerSecond,
                desired.angle.getRadians() - offsets[i],
                batteryVoltage,
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                DriveConstants.kModuleTranslations[i],
                0.02);
        }

        swerveDriveSim.update(forces, batteryVoltage, 0.02);

        if (navxYaw != null) {
            navxYaw.set(swerveDriveSim.getRotation2d().getDegrees());
        }

        m_odometry.update(
            swerveDriveSim.getRotation2d(),
            getModulePositions());

        if (m_field != null) {
            m_field.setRobotPose(getPose());
        }

        moduleStatePublisher.set(getModuleStates());

        // Publish actual chassis speeds (from module states in sim as well)
        var actual = DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
        actVxPublisher.set(actual.vxMetersPerSecond);
        actVyPublisher.set(actual.vyMetersPerSecond);
        actOmegaPublisher.set(actual.omegaRadiansPerSecond);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(getCurrentDraw()));
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

        ChassisSpeeds commanded = fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                                Rotation2d.fromDegrees(-m_gyro.getAngle()))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
        m_lastCommandedSpeeds = commanded;
        // Publish commanded chassis speeds
        cmdVxPublisher.set(commanded.vxMetersPerSecond);
        cmdVyPublisher.set(commanded.vyMetersPerSecond);
        cmdOmegaPublisher.set(commanded.omegaRadiansPerSecond);

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(commanded);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    public void driveChassisSpeeds(ChassisSpeeds speeds) {
        // System.out.println(speeds.vxMetersPerSecond + " " + speeds.vyMetersPerSecond
        // + " " + speeds.omegaRadiansPerSecond);
        m_lastCommandedSpeeds = speeds;
        // Publish commanded chassis speeds
        cmdVxPublisher.set(speeds.vxMetersPerSecond);
        cmdVyPublisher.set(speeds.vyMetersPerSecond);
        cmdOmegaPublisher.set(speeds.omegaRadiansPerSecond);

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setDesiredState(swerveModuleStates[i]);
        }
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
        for (int i = 0; i < m_modules.length; i++) {
            m_modules[i].setDesiredState(desiredStates[i]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
        for (int i = 0; i < m_modules.length; i++) {
            states[i] = m_modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[m_modules.length];
        for (int i = 0; i < m_modules.length; i++) {
            positions[i] = m_modules[i].getPosition();
        }
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
