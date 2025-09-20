package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PoseEstimatorSubsystem;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Factory for building the common drive commands used throughout the robot code.
 *
 * <p>The goal of this class is to give students a single place to look when they want to
 * understand how the driver sticks get translated into drivetrain motions. Every public
 * method returns a ready-to-run command that mirrors the legacy behaviors (manual drive,
 * heading locks, pose holds, etc.) so the rest of the robot code can request the right
 * behavior without duplicating logic. The helpers intentionally stay lightweight—each
 * command is built with a small lambda that calls {@link DriveSubsystem#drive} directly—so
 * it is easy to extend with future drive modes.
 */
public final class DriveCommandFactory {

    private final DriveSubsystem drive;
    private final PoseEstimatorSubsystem poseEstimator;
    private final XboxController driverController;
    private final PIDController rotPid;
    private final PIDController xPid;
    private final PIDController yPid;

    /**
     * Constructs a drive command factory that can create commands using the shared drivetrain,
     * pose estimator, and driver controller references.
     */
    public DriveCommandFactory(
            DriveSubsystem drive,
            PoseEstimatorSubsystem poseEstimator,
            XboxController driverController) {
        this.drive = Objects.requireNonNull(drive);
        this.poseEstimator = Objects.requireNonNull(poseEstimator);
        this.driverController = Objects.requireNonNull(driverController);
        this.rotPid = drive.getRotPidController();
        this.xPid = drive.getXPidController();
        this.yPid = drive.getYPidController();
    }

    /**
     * Small helper that bundles the driver stick inputs for a single loop of command execution.
     */
    private static final class DriverInputs {
        final double xSpeed;
        final double ySpeed;
        final double rotSpeed;

        DriverInputs(double xSpeed, double ySpeed, double rotSpeed) {
            this.xSpeed = xSpeed;
            this.ySpeed = ySpeed;
            this.rotSpeed = rotSpeed;
        }
    }

    /**
     * Samples the driver's left Y stick, applies the configured deadband, and flips the axis so
     * forward stick pushes produce positive field-relative X speeds.
     */
    private double sampleForwardInput() {
        return -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband);
    }

    /**
     * Samples the driver's left X stick with the same shaping as the original manual drive
     * command so small stick noise is ignored.
     */
    private double sampleStrafeInput() {
        return -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband);
    }

    /**
     * Samples the driver's right X stick to determine the desired rotational velocity.
     */
    private double sampleRotationInput() {
        return -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband);
    }

    /**
     * Grabs a snapshot of the driver inputs so each command can reason about the same numbers.
     */
    private DriverInputs sampleDriverInputs() {
        return new DriverInputs(sampleForwardInput(), sampleStrafeInput(), sampleRotationInput());
    }

    /**
     * Runs the provided controller each loop while reserving the drivetrain requirement.
     */
    private Command runDriveCommand(Consumer<DriverInputs> controller) {
        Objects.requireNonNull(controller);

        return Commands.run(() -> controller.accept(sampleDriverInputs()), drive);
    }

    /**
     * Applies manual rotation if the driver moves the right stick. Returns true when the manual
     * override has been applied so the caller can exit early.
     */
    private boolean applyManualRotationOverride(DriverInputs inputs) {
        if (inputs.rotSpeed != 0.0) {
            drive.drive(inputs.xSpeed, inputs.ySpeed, inputs.rotSpeed, true);
            return true;
        }
        return false;
    }

    /**
     * Creates a command that zeros all drivetrain outputs while scheduled.
     *
     * <p>This matches the previous {@code CancelledDrive} behavior and is used whenever the driver
     * state machine needs the drivetrain to coast.
     */
    public Command createCancelledCommand() {
        return Commands.run(() -> drive.drive(0.0, 0.0, 0.0, true), drive);
    }

    /**
     * Creates a command that simply passes driver input through to the drivetrain.
     *
     * <p>The stick shaping intentionally mirrors the legacy {@code ManualDrive} class so the student
     * controls feel identical after this refactor. This is the baseline mode used during teleop.
     */
    public Command createManualDriveCommand() {
        return runDriveCommand(inputs ->
                // Pass the field-relative speeds straight to the drivetrain.
                drive.drive(inputs.xSpeed, inputs.ySpeed, inputs.rotSpeed, true));
    }

    /**
     * Creates a command that mirrors the autonomous follower by steering toward the pose estimator's
     * target pose while still respecting manual rotation overrides.
     *
     * <p>This helper is used by the path-following state so students can compare the teleop
     * follower to the autonomous PathPlanner routine—it reuses the exact same PID controllers
     * that the autos rely on.
     */
    public Command createFollowPathCommand() {
        return runDriveCommand(inputs -> {
            // Give the driver full control of heading if they request it.
            if (applyManualRotationOverride(inputs)) {
                return;
            }

            // Otherwise steer toward the pose estimator's active trajectory target.
            Pose2d targetPose = Objects.requireNonNull(
                    poseEstimator.getTargetPose(), "Follow path target pose was null");

            // These PID calculations intentionally mirror the autonomous follower.
            double rotSpeed = rotPid.calculate(
                    poseEstimator.getDegrees(), targetPose.getRotation().getDegrees());
            double xCommand = xPid.calculate(poseEstimator.getCurrentPose().getX(), targetPose.getX());
            double yCommand = yPid.calculate(poseEstimator.getCurrentPose().getY(), targetPose.getY());

            // Drive toward the path planner target while still using field-relative speeds.
            drive.drive(xCommand, yCommand, rotSpeed, true);
        });
    }

    /**
     * Creates a command that keeps the robot pointed toward a heading unless overridden.
     *
     * <p>The caller supplies a heading provider so this helper can be used both for static targets
     * (like the stored barge heading) and dynamic targets (like pointing at a pose).
     */
    public Command createHeadingLockCommand(DoubleSupplier headingSupplier) {
        Objects.requireNonNull(headingSupplier);

        return runDriveCommand(inputs -> {
            // The student can override heading at any time with the right stick.
            if (applyManualRotationOverride(inputs)) {
                return;
            }

            double headingTarget = headingSupplier.getAsDouble();
            double rotSpeed = rotPid.calculate(poseEstimator.getDegrees(), headingTarget);
            drive.drive(inputs.xSpeed, inputs.ySpeed, rotSpeed, true);
        });
    }

    /**
     * Creates a command that aligns the robot to a pose while optionally blending in manual control.
     *
     * <p>The {@code manualBlend} parameter determines how much of the driver's translation input is
     * preserved while the PID controllers pull the robot onto the target pose. A blend of 0 keeps the
     * drivetrain fully locked to the target, while higher values allow slow creeping adjustments.
     */
    public Command createHoldPoseCommand(
            Supplier<Pose2d> targetPoseSupplier,
            double manualBlend,
            boolean updateTargetPose) {
        Objects.requireNonNull(targetPoseSupplier);

        return runDriveCommand(inputs -> {
            // Let the driver fully override heading control whenever they move the stick.
            if (applyManualRotationOverride(inputs)) {
                return;
            }

            Pose2d targetPose = Objects.requireNonNull(
                    targetPoseSupplier.get(), "Target pose supplier returned null");
            if (updateTargetPose) {
                // Synchronize the estimator's target pose so autonomous code still sees the same goal.
                poseEstimator.setTargetPose(targetPose);
            }

            // Drive the robot's X/Y toward the pose using the same PID controllers the autos use.
            double rotSpeed = rotPid.calculate(
                    poseEstimator.getDegrees(), targetPose.getRotation().getDegrees());
            double xCorrection =
                    xPid.calculate(poseEstimator.getCurrentPose().getX(), targetPose.getX());
            double yCorrection =
                    yPid.calculate(poseEstimator.getCurrentPose().getY(), targetPose.getY());

            // Blend in a fraction of the driver's manual input so they can creep around the target.
            double xCommand = xCorrection + inputs.xSpeed * manualBlend;
            double yCommand = yCorrection + inputs.ySpeed * manualBlend;
            drive.drive(xCommand, yCommand, rotSpeed, true);
        });
    }

    /**
     * Creates a command that keeps the robot aimed toward a target pose's translation while still
     * letting the driver translate.
     *
     * <p>This is essentially a heading lock where the heading is recomputed every cycle based on the
     * target pose. The drivetrain can still strafe manually, but its nose stays pointed at the
     * supplied pose—perfect for lining up with the reef or processor. When {@code updateTargetPose}
     * is {@code true} the pose estimator's public target is refreshed each loop so dashboards and
     * autos see the same heading the driver experiences.
     */
    public Command createPointingAtPoseCommand(
            Supplier<Pose2d> targetPoseSupplier,
            boolean updateTargetPose) {
        Objects.requireNonNull(targetPoseSupplier);

        // Delegate to the heading-lock helper so we keep all of the same manual override rules.
        return runDriveCommand(inputs -> {
            if (applyManualRotationOverride(inputs)) {
                return;
            }

            Pose2d targetPose = Objects.requireNonNull(
                    targetPoseSupplier.get(), "Target pose supplier returned null");
            double headingTarget = poseEstimator.turnToTarget(targetPose.getTranslation());
            if (updateTargetPose) {
                // Keep the estimator's public target pose synchronized for dashboards and autos.
                poseEstimator.setTargetPose(
                        new Pose2d(targetPose.getTranslation(), Rotation2d.fromDegrees(headingTarget)));
            }

            double rotSpeed = rotPid.calculate(poseEstimator.getDegrees(), headingTarget);
            drive.drive(inputs.xSpeed, inputs.ySpeed, rotSpeed, true);
        });
    }
}
