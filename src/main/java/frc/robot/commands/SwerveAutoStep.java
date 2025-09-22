// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.PoseEstimatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/** Generates a trajectory on the fly and feeds it to the pose estimator one sample at a time. */
public class SwerveAutoStep extends Command {
  PathPlannerPath path;
  PathPlannerTrajectory trajectory;

  PoseEstimatorSubsystem pose;

  Timer timer;

  /**
   * Creates a command that streams trajectory samples to the drive subsystem using the current
   * robot velocity as the starting condition.
   */
  public SwerveAutoStep(PathPlannerPath _path, PoseEstimatorSubsystem _pose) {
    path = _path;
    pose = _pose;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Build a trajectory using the current robot state as the starting conditions.
    trajectory =
        path.generateTrajectory(
            pose.getChassisSpeeds(), pose.getCurrentRotation(), pose.getRobotConfig());
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d target = trajectory.sample(timer.get()).pose;
    // Provide the desired pose to the drive state machine so it can follow the path.
    pose.setTargetPose(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    System.out.println("Time for " + path.name + ": " + timer.get());
    // Snap to the final pose to avoid drift if the drive command was interrupted early.
    pose.setTargetPose(trajectory.getEndState().pose);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() >= trajectory.getTotalTimeSeconds());
  }
}
