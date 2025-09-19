package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveStateMachine;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PoseEstimatorSubsystem;

/** Command used during autonomous path following to hold the driver input as overrides. */
public class FollowPathDrive extends Command{

    private DriveStateMachine stateMachine;
    private DriveSubsystem drive;
    private PoseEstimatorSubsystem poseEstimator;
    private XboxController driverController;

    // PID controllers that steer the robot back onto the planned path.
    private PIDController rotPid;
    private PIDController xPid;
    private PIDController yPid;

    // Cached path targets used to compute heading corrections each cycle.
    private double rotTarget = 0;
    private double rotSpeed = 0;
    private Pose2d targetPose = new Pose2d();

    /**
     * Creates a command that mirrors the autonomous path follower. It keeps the driver input
     * available as an override while PID loops steer toward the path planner target pose.
     */
    public FollowPathDrive(DriveSubsystem m_drive, PoseEstimatorSubsystem m_poseEstimator, XboxController m_driverController, DriveStateMachine m_driverMachine) {
        stateMachine = m_driverMachine;
        poseEstimator = m_poseEstimator;
        drive = m_drive;
        driverController = m_driverController;

        rotPid = drive.getRotPidController();
        xPid = drive.getXPidController();
        yPid = drive.getYPidController();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Called when the command is initially scheduled. Not used right now.
    @Override
    public void initialize() {
        // stateSubsystem.setDriveState(StateMachine.DriveState.REEF_RELATIVE);
        // rotPid.reset();
        // xPid.reset();
        // yPid.reset();
        // rotTarget = poseEstimator.getEstimatedPose().getRotation().getDegrees();
        // targetPose = poseEstimator.getEstimatedPose();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){

        // Get current controller inputs.
        double xPower = -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband);
        double yPower = -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband);
        double rotPower = -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband);

        // Rotation override else automatic angling.
        if (rotPower != 0) {
            drive.drive(
                xPower,
                yPower,
                rotPower,
                true);
        } else {

            targetPose = poseEstimator.getTargetPose(); // Target pose set by autos.
            rotTarget = targetPose.getRotation().getDegrees();
            rotSpeed = rotPid.calculate(poseEstimator.getDegrees(), rotTarget);
            xPower = xPid.calculate(poseEstimator.getCurrentPose().getX(), targetPose.getX());
            yPower = yPid.calculate(poseEstimator.getCurrentPose().getY(), targetPose.getY());

            //double vCmd = Math.sqrt(Math.pow(xPower, 2) + Math.pow(yPower, 2));
            //double heading = Math.atan2(yPower, xPower);
            //double modifier = slewLimiter.calculate(heading);

            //xPower = vCmd * Math.cos(modifier);
            //yPower = vCmd * Math.sin(modifier);

            // Drive toward the stored target pose while maintaining the requested heading.
            drive.drive(xPower, yPower, rotSpeed, true);
            //drive.drive(xFilter.calculate(xPower), yFilter.calculate(yPower), rotFilter.calculate(rotSpeed), true);
        }

    }

}
