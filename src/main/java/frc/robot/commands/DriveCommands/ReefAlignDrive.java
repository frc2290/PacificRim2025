package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateSubsystem.DriveState;
import frc.utils.PoseEstimatorSubsystem;

public class ReefAlignDrive extends Command {

    //imports
    private StateMachine stateMachine;
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
    public ReefAlignDrive(StateMachine m_state, DriveSubsystem m_drive, PoseEstimatorSubsystem m_poseEstimator, XboxController m_driverController) {

        stateMachine = m_state;
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

            targetPose = poseEstimator.getClosestBranch(stateMachine.getRightScore());
            poseEstimator.setTargetPose(targetPose);
            //targetPose = poseEstimator.getTargetPose();

            rotTarget = targetPose.getRotation().getDegrees();
            double tempRotPid = rotPid.calculate(poseEstimator.getDegrees(), rotTarget);
            rotSpeed = tempRotPid; //(poseEstimator.atTargetTheta() ? tempRotPid * 0.1 : tempRotPid);
            double xPowerPid = xPid.calculate(poseEstimator.getCurrentPose().getX(), targetPose.getX());
            double yPowerPid = yPid.calculate(poseEstimator.getCurrentPose().getY(), targetPose.getY());
            xPower = xPower * 0.5 + xPowerPid;//(poseEstimator.atTargetX() ? xPowerPid * 0.1 : xPowerPid);
            yPower = yPower * 0.5 + yPowerPid;//(poseEstimator.atTargetY(diff.hasLaserCanDistance()) ? yPowerPid * 0.1 : yPowerPid);

            drive.drive(xPower, yPower, rotSpeed, true);
            //drive.drive(xFilter.calculate(xPower), yFilter.calculate(yPower), rotFilter.calculate(rotSpeed), true);

            // LED Setting
            // if (poseEstimator.atTargetPose(diff.hasLaserCanDistance()) && stateSubsystem.atCurrentState()) {
            //     stateSubsystem.setDriveState(DriveState.ReefScore);
            // } else {
            //     stateSubsystem.setDriveState(DriveState.ReefScoreMove);
            // }
        }

    }

}
