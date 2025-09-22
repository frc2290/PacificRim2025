// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveStateMachine;
import frc.robot.subsystems.DriveStateMachine.DriveState;
import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateMachineCoardinator;
import frc.robot.subsystems.StateMachineCoardinator.RobotState;
import frc.utils.PoseEstimatorSubsystem;

/** Three piece left-side autonomous. Mirrors the right-side routine with corresponding paths. */
public class Left3Coral extends SequentialCommandGroup {

  public Left3Coral(
      PoseEstimatorSubsystem pose,
      DriveStateMachine driveState,
      StateMachineCoardinator coordinator,
      ManipulatorStateMachine manipulatorState,
      ManipulatorSubsystem manipulator) {
    try {
      PathPlannerPath startToReef = PathPlannerPath.fromPathFile("LeftCoral1");
      PathPlannerPath reefToFeeder = PathPlannerPath.fromPathFile("LeftCoral1ToFeeder");
      PathPlannerPath feederToReefTwo = PathPlannerPath.fromPathFile("FeederToLeftCoral2");
      PathPlannerPath reefTwoToFeeder = PathPlannerPath.fromPathFile("LeftCoral2ToFeeder");
      PathPlannerPath feederToReefThree = PathPlannerPath.fromPathFile("FeederToLeftCoral3");
      AutoRoutineFactory routineFactory =
          new AutoRoutineFactory(pose, coordinator, manipulatorState, manipulator);

      addCommands(
          // Match odometry with the starting point of the first path segment.
          Commands.runOnce(() -> pose.setCurrentPose(startToReef.getStartingHolonomicPose().get())),
          Commands.runOnce(
              () -> {
                // Enable path following and stage the manipulator in the safe configuration.
                driveState.setDriveCommand(DriveState.FOLLOW_PATH);
                coordinator.requestToScore(false);
                coordinator.setRobotGoal(RobotState.SAFE_CORAL_TRAVEL);
              }),
          // Perform three score-intake cycles on the left side of the field.
          routineFactory.scoreCoral(startToReef, RobotState.L4),
          routineFactory.intakeCoral(reefToFeeder),
          routineFactory.scoreCoral(feederToReefTwo, RobotState.L4),
          routineFactory.intakeCoral(reefTwoToFeeder),
          routineFactory.scoreCoral(feederToReefThree, RobotState.L4),
          // Return to the cancelled drive state so teleop can begin cleanly.
          Commands.runOnce(() -> driveState.setDriveCommand(DriveState.CANCELLED)));
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to build Left3Coral auto: " + ex.getMessage(), ex.getStackTrace());
      addCommands(Commands.none());
    }
  }
}
