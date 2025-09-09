package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveStateMachine;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PoseEstimatorSubsystem;


public class TeleopDrive extends Command {

    //imports
    private DriveStateMachine stateMachine;
    private DriveSubsystem drive;
    private PoseEstimatorSubsystem pose;
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
     * Command to just drive the with field oriented controls, default command for the the drive
     **/
    public TeleopDrive(DriveSubsystem m_drive, PoseEstimatorSubsystem m_pose, XboxController m_driverController, DriveStateMachine m_driverMachine) {

        stateMachine = m_driverMachine;
        pose = m_pose;
        drive = m_drive;
        driverController = m_driverController;

        //rotPid = pose.getRotPidController();
        //xPid = pose.getXPidController();
        //yPid = pose.getYPidController();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Called when the command is initially scheduled. Not used right now
    @Override   
    public void initialize() {
        // stateSubsystem.setDriveState(StateMachine.DriveState.TELEOP);
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

        //Square inputs for finer control at low speeds TEST OUT LATER
        //xPower = Math.copySign(xPower * xPower, xPower);

        drive.drive(xPower, yPower, rotPower, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
