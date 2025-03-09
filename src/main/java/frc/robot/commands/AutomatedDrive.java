// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.sql.rowset.RowSetProvider;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.DriveState;
import frc.utils.LEDEffects;
import frc.utils.LEDUtility;
import frc.utils.PoseEstimatorSubsystem;
import frc.utils.LEDEffects.LEDEffect;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutomatedDrive extends Command {

    private StateSubsystem stateSubsystem;
    private DriveSubsystem drive;
    private PoseEstimatorSubsystem poseEstimator;
    private XboxController driverController;

    private PIDController rotPid = new PIDController(0.01, 0.001, 0.001);
    private PIDController xPid = new PIDController(0.6, 0.0075, 0.1);
    private PIDController yPid = new PIDController(0.6, 0.0075, 0.1);
    
    private double rotTarget = 0;
    private double rotSpeed = 0;
    private Pose2d targetPose = new Pose2d();

    /** Creates a new AutomatedDrive. */
    public AutomatedDrive(StateSubsystem m_state, DriveSubsystem m_drive, PoseEstimatorSubsystem m_pose, XboxController m_driverController) {
        stateSubsystem = m_state;
        drive = m_drive;
        poseEstimator = m_pose;
        driverController = m_driverController;

        rotPid.enableContinuousInput(0, 360);

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

            rotTarget = targetPose.getRotation().getDegrees();
            rotSpeed = rotPid.calculate(poseEstimator.getDegrees(), rotTarget);
            xPower = xPower * 0.5 + xPid.calculate(poseEstimator.getCurrentPose().getX(), targetPose.getX());
            yPower = yPower * 0.5 + yPid.calculate(poseEstimator.getCurrentPose().getY(), targetPose.getY());

            drive.drive(xPower, yPower, rotSpeed, true);

            // LED Setting
            if (poseEstimator.getAlignX(targetPose.getTranslation()) < 0.01 && poseEstimator.getAlignY(targetPose.getTranslation()) < 0.01 && stateSubsystem.atCurrentState()) {
                stateSubsystem.setDriveState(DriveState.ReefScore);
            }
        }
        // Teleop state: If we have a coral, point robot towards center of reef
        else if (stateSubsystem.getDriveState() == DriveState.Teleop) {
            rotTarget = poseEstimator.turnToTarget(VisionConstants.reefCenter);
            rotSpeed = rotPid.calculate(poseEstimator.getDegrees(), rotTarget);

            drive.drive(xPower, yPower, rotSpeed, true);
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
