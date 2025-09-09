package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveStateManager;
import frc.utils.PoseEstimatorSubsystem;

public class FollowPathDrive extends Command{

        //imports
    private DriveStateManager driveStateManager;
    private DriveSubsystem drive;
    private PoseEstimatorSubsystem poseEstimator;
    private XboxController driverController;

    //pid
    private PIDController rotPid;
    private PIDController xPid;
    private PIDController yPid;

    //pos estimator
    private double rotTarget = 0;
    private double rotSpeed = 0;
    private Pose2d targetPose = new Pose2d();

    /*
     * Command to align neareest reef (usually has note)
     **/
    public FollowPathDrive(DriveStateManager m_state, DriveSubsystem m_drive, PoseEstimatorSubsystem m_poseEstimator, XboxController m_driverController) {

        driveStateManager = m_state;
        drive = m_drive;
        poseEstimator = m_poseEstimator;
        driverController = m_driverController;

        rotPid = m_drive.getRotPidController();
        xPid = m_drive.getXPidController();
        yPid = m_drive.getYPidController();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drive);
    }

    // Called when the command is initially scheduled. Not used right now
    @Override
    public void initialize() {
        // stateSubsystem.setDriveState(DriveState.ReefRelative);
        // rotPid.reset();
        // xPid.reset();
        // yPid.reset();
        // rotTarget = poseEstimator.getEstimatedPose().getRotation().getDegrees();
        // targetPose = poseEstimator.getEstimatedPose();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){

        //Get current controller inputs
        double xPower = -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband);
        double yPower = -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband);
        double rotPower = -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband);

        //Rotation override else automatic angling
        if (rotPower != 0) {
            drive.drive(
                xPower,
                yPower,
                rotPower,
                true);
        }else{
            
            targetPose = poseEstimator.getTargetPose(); // Target pose set by autos
            rotTarget = targetPose.getRotation().getDegrees();
            rotSpeed = rotPid.calculate(poseEstimator.getDegrees(), rotTarget);
            xPower = xPid.calculate(poseEstimator.getCurrentPose().getX(), targetPose.getX());
            yPower = yPid.calculate(poseEstimator.getCurrentPose().getY(), targetPose.getY());

            //double vCmd = Math.sqrt(Math.pow(xPower, 2) + Math.pow(yPower, 2));
            //double heading = Math.atan2(yPower, xPower);
            //double modifier = slewLimiter.calculate(heading);

            //xPower = vCmd * Math.cos(modifier);
            //yPower = vCmd * Math.sin(modifier);

            drive.drive(xPower, yPower, rotSpeed, true);
            //drive.drive(xFilter.calculate(xPower), yFilter.calculate(yPower), rotFilter.calculate(rotSpeed), true);
        }

    }

}
