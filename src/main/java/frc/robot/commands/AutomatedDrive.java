// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.DriveState;
import frc.utils.PoseEstimatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutomatedDrive extends Command {

    private StateSubsystem stateSubsystem;
    private DriveSubsystem drive;
    private PoseEstimatorSubsystem poseEstimator;
    private XboxController driverController;

    private PIDController rotPid;
    private PIDController xPid;
    private PIDController yPid;

    private SlewRateLimiter slewLimiter = new SlewRateLimiter(150);
    
    private double rotTarget = 0;
    private double rotSpeed = 0;
    private Pose2d targetPose = new Pose2d();

    /** Creates a new AutomatedDrive. */
    public AutomatedDrive(StateSubsystem m_state, DriveSubsystem m_drive, PoseEstimatorSubsystem m_pose, XboxController m_driverController) {
        stateSubsystem = m_state;
        drive = m_drive;
        poseEstimator = m_pose;
        driverController = m_driverController;

        rotPid = m_drive.getRotPidController();
        xPid = m_drive.getXPidController();
        yPid = m_drive.getYPidController();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get current controller inputs
        double xPower = -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband);
        double yPower = -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband);
        double rotPower = -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband);

        // If driver rotation stick is moved or rotation lock is off, let driver move robot
        if (rotPower != 0 || !stateSubsystem.getRotationLock()) {
            drive.drive(
                xPower,
                yPower,
                rotPower,
                true);
        }
        // Coral station state: Rotate rear of robot to face whichever coral station is closest
        else if (stateSubsystem.getDriveState() == DriveState.CoralStation) {
            // Rotate to whichever coral station is closest
            rotTarget = poseEstimator.isClosestStationRight() ? VisionConstants.coralStationRightHeading : VisionConstants.coralStationLeftHeading;
            rotSpeed = rotPid.calculate(poseEstimator.getDegrees(), rotTarget);

            drive.drive(xPower, yPower, rotSpeed, true);
        }
        // Reef state: Move robot to nearest branch for scoring
        else if (stateSubsystem.getDriveState() == DriveState.ReefScore || stateSubsystem.getDriveState() == DriveState.ReefScoreMove) {
            targetPose = poseEstimator.getClosestBranch(stateSubsystem.getRightScore());
            poseEstimator.setTargetPose(targetPose);
            //targetPose = poseEstimator.getTargetPose();

            rotTarget = targetPose.getRotation().getDegrees();
            double tempRotPid = rotPid.calculate(poseEstimator.getDegrees(), rotTarget);
            rotSpeed = (poseEstimator.atTargetTheta() ? tempRotPid * 0.1 : tempRotPid);
            double xPowerPid = xPid.calculate(poseEstimator.getCurrentPose().getX(), targetPose.getX());
            double yPowerPid = yPid.calculate(poseEstimator.getCurrentPose().getY(), targetPose.getY());
            xPower = xPower * 0.5 + (poseEstimator.atTargetX() ? xPowerPid * 0.1 : xPowerPid);
            yPower = yPower * 0.5 + (poseEstimator.atTargetY() ? yPowerPid * 0.1 : yPowerPid);

            //yPowerPid = yPowerPid - (rotSpeed * 0.7);

            //double vCmd = Math.sqrt(Math.pow(xPowerPid, 2) + Math.pow(yPowerPid, 2));
            //double heading = Math.atan2(yPowerPid, xPowerPid);
            //double modifier = slewLimiter.calculate(heading);

            //xPower = xPower * 0.5 + (vCmd * Math.cos(modifier));
            //yPower = yPower * 0.5 + (vCmd * Math.sin(modifier));

            drive.drive(xPower, yPower, rotSpeed, true);

            // LED Setting
            if (poseEstimator.atTargetPose() && stateSubsystem.atCurrentState()) {
                stateSubsystem.setDriveState(DriveState.ReefScore);
            } else {
                stateSubsystem.setDriveState(DriveState.ReefScoreMove);
            }
        }
        // Teleop state: If we have a coral, point robot towards center of reef
        else if (stateSubsystem.getDriveState() == DriveState.Teleop) {
            rotTarget = poseEstimator.turnToTarget(VisionConstants.reefCenter);
            rotSpeed = rotPid.calculate(poseEstimator.getDegrees(), rotTarget);

            drive.drive(xPower, yPower, rotSpeed, true);
        } 
        // Auto state: Read target state from pose estimator that is fed from auto routine and drive to it. Will hold us there too until it changes
        else if (stateSubsystem.getDriveState() == DriveState.Auto) {
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
        } 
        // Climb state: Implement
        else if (stateSubsystem.getDriveState() == DriveState.Climb) {
            // Implement
        }
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
