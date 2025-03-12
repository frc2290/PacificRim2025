// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.PositionState;
import frc.utils.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Right1Coral extends SequentialCommandGroup {
    /** Creates a new Right1Coral. */
    public Right1Coral(PoseEstimatorSubsystem poseEst, StateSubsystem stateSubsystem, Command scoreCommand) {
        try {
            // Pull in path from start location to reef
            PathPlannerPath startToReef = PathPlannerPath.fromPathFile("RightCoral1");
            // Create a reset pose command to set starting location (may remove in future)
            Command resetPose = new InstantCommand(() -> poseEst.setCurrentPose(startToReef.getStartingDifferentialPose()));
            // Create a command to go to level 4 score position
            Command goToL4 = stateSubsystem.setGoalCommandTest(PositionState.L4Position);
            //goToL4.onlyWhile(() -> poseEst.getCurrentPose().getTranslation().getDistance(VisionConstants.reefCenter) < 3);
            // Create a parallel group to move to the reef and get in scoring position at the same time
            Command moveToReef = new ParallelCommandGroup(AutoBuilder.followPath(startToReef), goToL4);

            // Add your commands in the addCommands() call, e.g.
            // addCommands(new FooCommand(), new BarCommand());
            // First reset position, move to reef and get in score position, lastly score the coral
            addCommands(resetPose, moveToReef, scoreCommand);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            Commands.none();
        }
    }
}
