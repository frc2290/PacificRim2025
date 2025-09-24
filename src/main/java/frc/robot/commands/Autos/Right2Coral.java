// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveStateMachine;
import frc.robot.subsystems.DriveStateMachine.DriveState;
import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateMachineCoordinator;
import frc.robot.subsystems.StateMachineCoordinator.RobotState;
import frc.utils.PoseEstimatorSubsystem;

/**
 * Two piece autonomous starting on the right side: scores, loads from the feeder, and scores again.
 */
public class Right2Coral extends SequentialCommandGroup {

  public Right2Coral(
      PoseEstimatorSubsystem pose,
      DriveStateMachine driveState,
      StateMachineCoordinator coordinator,
      ManipulatorStateMachine manipulatorState,
      ManipulatorSubsystem manipulator) {
    try {
      PathPlannerPath startToReef = PathPlannerPath.fromPathFile("RightCoral1");
      PathPlannerPath reefToFeeder = PathPlannerPath.fromPathFile("RightCoral1ToFeeder");
      PathPlannerPath feederToReef = PathPlannerPath.fromPathFile("FeederToRightCoral2");
      AutoRoutineFactory routineFactory =
          new AutoRoutineFactory(pose, coordinator, manipulatorState, manipulator);

      addCommands(
          // Align robot odometry with the path's starting pose.
          Commands.runOnce(() -> pose.setCurrentPose(startToReef.getStartingHolonomicPose().get())),
          Commands.runOnce(
              () -> {
                // Enable path following and tell the manipulator that we intend to score first.
                driveState.setDriveCommand(DriveState.FOLLOW_PATH);
                coordinator.requestToScore(false);
                coordinator.setRobotGoal(RobotState.SAFE_CORAL_TRANSPORT);
              }),
          // Score the preloaded coral, grab another from the feeder, then score again.
          routineFactory.scoreCoral(startToReef, RobotState.L4),
          routineFactory.intakeCoral(reefToFeeder),
          routineFactory.scoreCoral(feederToReef, RobotState.L4),
          // Leave the drive state machine in a neutral configuration for teleop.
          Commands.runOnce(() -> driveState.setDriveCommand(DriveState.CANCELLED)));
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to build Right2Coral auto: " + ex.getMessage(), ex.getStackTrace());
      addCommands(Commands.none());
    }
  }
}
