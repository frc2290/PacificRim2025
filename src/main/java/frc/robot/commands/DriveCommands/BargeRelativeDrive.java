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

public class BargeRelativeDrive extends Command {

  // imports
  private DriveStateMachine stateMachine;
  private DriveSubsystem drive;
  private PoseEstimatorSubsystem poseEstimator;
  private XboxController driverController;

  // pid
  private PIDController rotPid;
  private PIDController xPid;
  private PIDController yPid;

  // pos estimator
  private double rotTarget = 0;
  private double rotSpeed = 0;
  private Pose2d targetPose = new Pose2d();

  /*
   * Command to align neareest reef (usually has note)
   **/
  public BargeRelativeDrive(
      DriveSubsystem m_drive,
      PoseEstimatorSubsystem m_poseEstimator,
      XboxController m_driverController,
      DriveStateMachine m_driverMachine) {
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
  public void execute() {

    // Get current controller inputs
    double xPower =
        -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband);
    double yPower =
        -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband);
    double rotPower =
        -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband);

    // //Rotation override else automatic angling
    // if (rotPower != 0) {
    //     drive.drive(
    //         xPower,
    //         yPower,
    //         rotPower,
    //         true);
    // }else{
    //     //make it so robot always faces towards barge and stops at one of the 3 center points
    // positions from which it can score, first implement one
    //     rotTarget = poseEstimator.turnToTarget(VisionConstants.reefCenter);
    //     rotSpeed = rotPid.calculate(poseEstimator.getDegrees(), rotTarget);
    //     drive.drive(xPower, yPower, rotSpeed, true);
    // }

  }
}
