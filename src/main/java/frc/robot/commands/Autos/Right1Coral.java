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
import frc.robot.subsystems.StateMachineCoardinator;
import frc.robot.subsystems.StateMachineCoardinator.RobotState;
import frc.utils.PoseEstimatorSubsystem;

/** One piece autonomous that drives to the right branch and scores a single coral. */
public class Right1Coral extends SequentialCommandGroup {

  public Right1Coral(
      PoseEstimatorSubsystem pose,
      DriveStateMachine driveState,
      StateMachineCoardinator coordinator,
      ManipulatorStateMachine manipulatorState,
      ManipulatorSubsystem manipulator) {
    try {
      PathPlannerPath startToReef = PathPlannerPath.fromPathFile("RightCoral1");
      AutoRoutineFactory routineFactory =
          new AutoRoutineFactory(pose, coordinator, manipulatorState, manipulator);

      addCommands(
          // Snap the pose estimator to the first point of the autonomous path.
          Commands.runOnce(() -> pose.setCurrentPose(startToReef.getStartingHolonomicPose().get())),
          Commands.runOnce(
              () -> {
                // Enable path following and prep the manipulator for scoring.
                driveState.setDriveCommand(DriveState.FOLLOW_PATH);
                coordinator.requestToScore(false);
                coordinator.setRobotGoal(RobotState.SAFE_CORAL_TRAVEL);
              }),
          // Drive out and score the preloaded coral, then stop path following.
          routineFactory.scoreCoral(startToReef, RobotState.L4),
          // Ensure the drivetrain defaults back to manual control.
          Commands.runOnce(() -> driveState.setDriveCommand(DriveState.CANCELLED)));
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to build Right1Coral auto: " + ex.getMessage(), ex.getStackTrace());
      addCommands(Commands.none());
    }
  }
}
