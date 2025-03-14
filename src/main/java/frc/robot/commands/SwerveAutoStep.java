// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.PoseEstimatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveAutoStep extends Command {
    PathPlannerPath path;
    PathPlannerTrajectory trajectory;

    PoseEstimatorSubsystem pose;

    Timer timer;

    /** Creates a new SwerveAutoCommand. */
    public SwerveAutoStep(PathPlannerPath _path, PoseEstimatorSubsystem _pose) {
        path = _path;
        pose = _pose;
        trajectory = path.generateTrajectory(pose.getChassisSpeeds(), pose.getCurrentRotation(), pose.getRobotConfig());
        timer = new Timer();
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d target = trajectory.sample(timer.get()).pose;
        pose.setTargetPose(target);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        pose.setTargetPose(trajectory.getEndState().pose);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (timer.get() >= trajectory.getTotalTimeSeconds());
    }
}
