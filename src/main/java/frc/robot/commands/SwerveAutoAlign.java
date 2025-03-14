// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.DriveState;
import frc.utils.PoseEstimatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveAutoAlign extends Command {
    private PoseEstimatorSubsystem pose;
    private StateSubsystem state;

    private Pose2d target;
    private PathPlannerTrajectory trajectory;

    private Timer timer;

    /** Creates a new SwerveAutoAlign. */
    public SwerveAutoAlign(PoseEstimatorSubsystem _pose, StateSubsystem _state) {
        pose = _pose;
        state = _state;
        timer = new Timer();
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        state.setDriveState(DriveState.ReefScoreMove);

        Pose2d nearestBranch = pose.getClosestBranch(state.getRightScore());

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(pose.getCurrentPose().getTranslation(), getPathVelocityHeading(pose.getChassisSpeeds(), nearestBranch)),
            nearestBranch);

        PathConstraints constraints = AutoConstants.kPathConstraints;

        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            constraints,
            new IdealStartingState(getVelocityMagnitude(pose.getChassisSpeeds()), pose.getCurrentRotation()), 
            new GoalEndState(0.0, nearestBranch.getRotation())
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        trajectory = path.generateTrajectory(pose.getChassisSpeeds(), pose.getCurrentRotation(), pose.getRobotConfig());

        timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        target = trajectory.sample(timer.get()).pose;
        pose.setTargetPose(target);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (timer.get() >= trajectory.getTotalTimeSeconds());
    }

    /**
     * 
     * @param cs field relative chassis speeds
     * @return
     */
    private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
        if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
            var diff = target.minus(pose.getCurrentPose()).getTranslation();
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();//.rotateBy(Rotation2d.k180deg);
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs){
        return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
    }
}
