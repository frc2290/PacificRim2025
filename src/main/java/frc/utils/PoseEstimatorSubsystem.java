package frc.utils;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;

/**
 * Pose estimator that uses odometry and AprilTags with PhotonVision.
 */
public class PoseEstimatorSubsystem extends SubsystemBase {

    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
     * meters.
     */
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.01); //VecBuilder.fill(0.1, 0.1, 0.1);

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.25, 0.25, 0.1); //VecBuilder.fill(1.0, 1.0, 1.0);

    private final Supplier<Rotation2d> rotationSupplier;
    private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
    private final Supplier<SwerveModuleState[]> moduleStateSupplier;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field2d = new Field2d();
    private final FieldObject2d target2d = field2d.getObject("Target");
    private final PhotonRunnable photonEstimator;
    private final PhotonRunnable photonEstimator2;
    private final Notifier photonNotifier;
    private final Notifier photonNotifier2;

    private OriginPosition originPosition = kBlueAllianceWallRightSide;
    private boolean sawTag = false;

    private RobotConfig config;

    private Pose2d targetPose = new Pose2d();

    private DriveSubsystem drive;

    private FlytLogger poseDash = new FlytLogger("Pose");

    public PoseEstimatorSubsystem(DriveSubsystem m_drive) {
        drive = m_drive;
        photonEstimator = new PhotonRunnable("FrontCamera", VisionConstants.APRILTAG_CAMERA_TO_ROBOT);
        photonEstimator2 = new PhotonRunnable("RearCamera", VisionConstants.APRILTAG_CAMERA2_TO_ROBOT);

        photonNotifier = new Notifier(photonEstimator);
        photonNotifier2 = new Notifier(photonEstimator2);

        this.rotationSupplier = m_drive::newHeading;
        this.modulePositionSupplier = m_drive::getModulePositions;
        this.moduleStateSupplier = m_drive::getModuleStates;

        poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.kDriveKinematics,
                rotationSupplier.get(),
                modulePositionSupplier.get(),
                new Pose2d(),
                stateStdDevs,
                visionMeasurementStdDevs);

        // Start PhotonVision thread
        photonNotifier.setName("PhotonRunnable");
        photonNotifier.startPeriodic(0.01);
        photonNotifier2.setName("PhotonRunnable2");
        photonNotifier2.startPeriodic(0.01);

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getCurrentPose, // Robot pose supplier
                this::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (ChassisSpeeds speeds) -> m_drive.driveChassisSpeeds(new ChassisSpeeds(speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond)), // Method that will drive the robot
                                                                                  // given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(18.0, 0.0, 0.18), // Translation PID constants
                        new PIDConstants(14.0, 0.0, 0.18) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                m_drive // Reference to this subsystem to set requirements
        );

        poseDash.addStringPublisher("Pose", false, () -> getCurrentPose().toString());
        poseDash.addBoolPublisher("At Target Pose", false, () -> atTargetPose());
        poseDash.addDoublePublisher("Relative To X", true, () -> poseEstimator.getEstimatedPosition().relativeTo(targetPose).getX());
        poseDash.addDoublePublisher("Relative To Y", true, () -> poseEstimator.getEstimatedPosition().relativeTo(targetPose).getY());
        poseDash.addDoublePublisher("Relative To T", true, () -> poseEstimator.getEstimatedPosition().relativeTo(targetPose).getRotation().getDegrees());
    }

    /**
     * Sets the alliance. This is used to configure the origin of the AprilTag map
     * 
     * @param alliance alliance
     */
    public void setAlliance(Alliance alliance) {
        boolean allianceChanged = false;
        switch (alliance) {
            case Blue:
                allianceChanged = (originPosition == kRedAllianceWallRightSide);
                originPosition = kBlueAllianceWallRightSide;
                break;
            case Red:
                allianceChanged = (originPosition == kBlueAllianceWallRightSide);
                originPosition = kRedAllianceWallRightSide;
                break;
            default:
                // No valid alliance data. Nothing we can do about it
        }

        if (allianceChanged && sawTag) {
            // The alliance changed, which changes the coordinate system.
            // Since a tag was seen, and the tags are all relative to the coordinate system,
            // the estimated pose
            // needs to be transformed to the new coordinate system.
            var newPose = flipAlliance(getCurrentPose());
            poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
        }
    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors
        poseEstimator.update(rotationSupplier.get(), modulePositionSupplier.get());

        var visionPose = photonEstimator.grabLatestEstimatedPose();
        if (visionPose != null) {
            // New pose from vision
            sawTag = true;
            var pose2d = visionPose.estimatedPose.toPose2d();
            if (originPosition != kBlueAllianceWallRightSide) {
                pose2d = flipAlliance(pose2d);
            }
            //if (!DriverStation.isAutonomous() || (poseEstimator.getEstimatedPosition().getTranslation().getDistance(VisionConstants.reefCenter) < 3)) {
                poseEstimator.addVisionMeasurement(pose2d, visionPose.timestampSeconds);
            //}
        }

        var visionPose2 = photonEstimator2.grabLatestEstimatedPose();
        if (visionPose2 != null) {
            // New pose from vision
            sawTag = true;
            var pose2d2 = visionPose2.estimatedPose.toPose2d();
            if (originPosition != kBlueAllianceWallRightSide) {
                pose2d2 = flipAlliance(pose2d2);
            }
            //if (PhotonUtils.getDistanceToPose(getCurrentPose(), photonEstimator2.grabLatestResult()) < 3) {
                poseEstimator.addVisionMeasurement(pose2d2, visionPose2.timestampSeconds);
            //}
        }

        // Set the pose on the dashboard
        var dashboardPose = poseEstimator.getEstimatedPosition();
        if (originPosition == kRedAllianceWallRightSide) {
            // Flip the pose when red, since the dashboard field photo cannot be rotated
            dashboardPose = flipAlliance(dashboardPose);
        }
        field2d.setRobotPose(dashboardPose);
        target2d.setPose(targetPose);
        SmartDashboard.putData("Field", field2d);
        poseDash.update(Constants.debugMode);
    }

    private String getFomattedPose() {
        var pose = getCurrentPose();
        return String.format("(%.3f, %.3f) %.2f degrees",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees());
    }

    public boolean isClosestStationRight() {
        if (poseEstimator.getEstimatedPosition().getY() > VisionConstants.halfwayAcrossFieldY) {
            return false;
        } else {
            return true;
        }
    }

    public Pose2d getClosestBranch(boolean right) {
        if (right) {
            return poseEstimator.getEstimatedPosition().nearest(VisionConstants.rightBranches);
        } else {
            return poseEstimator.getEstimatedPosition().nearest(VisionConstants.leftBranches);
        }
    }

    public double getAlignX(Translation2d target) {
        return target.getX() - poseEstimator.getEstimatedPosition().getX();
    }

    public double getAlignY(Translation2d target) {
        return target.getY() - poseEstimator.getEstimatedPosition().getY();
    }

    public double turnToTarget(Translation2d target) {
        double offsetX = target.getX() - poseEstimator.getEstimatedPosition().getX();
        double offsetY = target.getY() - poseEstimator.getEstimatedPosition().getY();
        //return (360 - Math.toDegrees(Math.atan2(offsetY, offsetX)) % 360);
        return (Math.toDegrees(Math.atan2(offsetY, offsetX)) % 360);
    }

    public double getDegrees() {
        return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }

    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public PhotonPipelineResult getLatestTag() {
        return photonEstimator2.grabLatestTag();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(moduleStateSupplier.get());
    }

    public Rotation2d getCurrentRotation() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public RobotConfig getRobotConfig() {
        return config;
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    public void setTargetPose(Pose2d newTarget) {
        targetPose = newTarget;
    }

    public Command setTargetPoseCommand(Pose2d newTarget) {
        return Commands.runOnce(() -> setTargetPose(newTarget));
    }

    public boolean atTargetPose() {
        Pose2d relative = getCurrentPose().relativeTo(targetPose);
        return Math.abs(relative.getX()) < 0.0254
                && Math.abs(relative.getY()) < 0.06
                && Math.abs(relative.getRotation().getDegrees()) < 2;
        //return Math.abs(getAlignX(targetPose.getTranslation())) < 0.03
         //       && Math.abs(getAlignY(targetPose.getTranslation())) < 0.03
         //       && Math.abs(poseEstimator.getEstimatedPosition().getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < 2;
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     * 
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
        //drive.setGyroAdjustment(newPose.getRotation().getDegrees());
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being
     * downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    /**
     * Transforms a pose to the opposite alliance's coordinate system. (0,0) is
     * always on the right corner of your
     * alliance wall, so for 2023, the field elements are at different coordinates
     * for each alliance.
     * 
     * @param poseToFlip pose to transform to the other alliance
     * @return pose relative to the other alliance's coordinate system
     */
    private Pose2d flipAlliance(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(VisionConstants.FLIPPING_POSE);
    }
}