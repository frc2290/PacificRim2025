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

/** Three piece right-side autonomous. Paths assume a feeder pickup in between each score. */
public class Right3Coral extends SequentialCommandGroup {

  public Right3Coral(
      PoseEstimatorSubsystem pose,
      DriveStateMachine driveState,
      StateMachineCoardinator coordinator,
      ManipulatorStateMachine manipulatorState,
      ManipulatorSubsystem manipulator) {
    try {
      PathPlannerPath startToReef = PathPlannerPath.fromPathFile("RightCoral1");
      PathPlannerPath reefToFeeder = PathPlannerPath.fromPathFile("RightCoral1ToFeeder");
      PathPlannerPath feederToReefTwo = PathPlannerPath.fromPathFile("FeederToRightCoral2");
      PathPlannerPath reefTwoToFeeder = PathPlannerPath.fromPathFile("RightCoral2ToFeeder");
      PathPlannerPath feederToReefThree = PathPlannerPath.fromPathFile("FeederToRightCoral3");
      AutoRoutineFactory routineFactory =
          new AutoRoutineFactory(pose, coordinator, manipulatorState, manipulator);

      addCommands(
          // Lock odometry to match the starting pose of the generated path.
          Commands.runOnce(() -> pose.setCurrentPose(startToReef.getStartingHolonomicPose().get())),
          Commands.runOnce(
              () -> {
                // Start path following and make sure the manipulator begins in the safe pose.
                driveState.setDriveCommand(DriveState.FOLLOW_PATH);
                coordinator.requestToScore(false);
                coordinator.setRobotGoal(RobotState.SAFE_CORAL_TRANSPORT);
              }),
          // Score, reload, and score two additional corals.
          routineFactory.scoreCoral(startToReef, RobotState.L4),
          routineFactory.intakeCoral(reefToFeeder),
          routineFactory.scoreCoral(feederToReefTwo, RobotState.L4),
          routineFactory.intakeCoral(reefTwoToFeeder),
          routineFactory.scoreCoral(feederToReefThree, RobotState.L4),
          // Return drivetrain control to the driver when auto finishes.
          Commands.runOnce(() -> driveState.setDriveCommand(DriveState.CANCELLED)));
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to build Right3Coral auto: " + ex.getMessage(), ex.getStackTrace());
      addCommands(Commands.none());
    }
  }
}
