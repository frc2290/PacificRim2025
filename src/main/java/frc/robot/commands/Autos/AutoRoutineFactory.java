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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveAutoStep;
import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorStateMachine.ElevatorManipulatorState;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateMachineCoordinator;
import frc.robot.subsystems.StateMachineCoordinator.RobotState;
import frc.utils.PoseEstimatorSubsystem;

/**
 * Utility that assembles the common manipulator/drive command sequences shared across the autos.
 */
public class AutoRoutineFactory {

  private final PoseEstimatorSubsystem pose;
  private final StateMachineCoordinator coordinator;
  private final ManipulatorStateMachine manipulatorState;
  private final ManipulatorSubsystem manipulator;

  /** Creates a helper that can be reused by each autonomous routine to share core steps. */
  public AutoRoutineFactory(
      PoseEstimatorSubsystem pose,
      StateMachineCoordinator coordinator,
      ManipulatorStateMachine manipulatorState,
      ManipulatorSubsystem manipulator) {
    this.pose = pose;
    this.coordinator = coordinator;
    this.manipulatorState = manipulatorState;
    this.manipulator = manipulator;
  }

  /** Builds the repeated scoring sequence used by the one-, two-, and three-piece routines. */
  public Command scoreCoral(PathPlannerPath path, RobotState targetState) {
    return new SequentialCommandGroup(
        Commands.runOnce(
            () -> {
              // Tell the manipulator state machine which goal we are chasing.
              manipulatorState.atGoalState(false);
              coordinator.setRobotGoal(targetState);
            }),
        // Drive to the reef while the manipulator moves in parallel.
        Commands.parallel(new SwerveAutoStep(path, pose), manipulatorState.waitUntilReady()),
        // Once staged, request the scoring action.
        Commands.runOnce(() -> coordinator.requestToScore(true)),
        // Wait for the manipulator to confirm the coral has been released before cleaning up.
        Commands.waitUntil(() -> !manipulator.hasCoral()),
        Commands.runOnce(
            () -> {
              // Reset to a safe travel configuration after scoring completes.
              coordinator.requestToScore(false);
              coordinator.setRobotGoal(RobotState.SAFE_CORAL_TRANSPORT);
            }),
        manipulatorState.waitForState(ElevatorManipulatorState.SAFE_CORAL_TRAVEL));
  }

  /** Builds the feeder intake sequence shared by the multi-piece routines. */
  public Command intakeCoral(PathPlannerPath path) {
    return new SequentialCommandGroup(
        Commands.runOnce(
            () -> {
              // Leave scoring mode and configure for intake.
              manipulatorState.atGoalState(false);
              coordinator.requestToScore(false);
              coordinator.setRobotGoal(RobotState.INTAKE_CORAL);
            }),
        // Path to the station while waiting for the beam break to trip.
        Commands.parallel(
            new SwerveAutoStep(path, pose), Commands.waitUntil(() -> manipulator.hasCoral())),
        manipulatorState.waitForState(ElevatorManipulatorState.INTAKE_CORAL),
        // Once a coral is detected, command the manipulator to stow safely again.
        Commands.runOnce(() -> coordinator.setRobotGoal(RobotState.SAFE_CORAL_TRANSPORT)),
        manipulatorState.waitForState(ElevatorManipulatorState.SAFE_CORAL_TRAVEL));
  }
}
