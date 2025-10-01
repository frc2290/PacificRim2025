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
package frc.robot.commands.ElevatorManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorManipulatorPositions.ManipulatorPosition;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;

/** Factory for creating manipulator position commands. */
public final class ManipulatorPositionCommandFactory {

  private ManipulatorPositionCommandFactory() {}

  /**
   * Creates a command that drives the elevator and differential arm to the supplied setpoints
   * without marking the manipulator state machine as ready at the end of the sequence.
   */
  public static Command createPrepCommand( // Intermediate pos command
      ManipulatorStateMachine manipulatorState,
      DifferentialSubsystem diffArm,
      ElevatorSubsystem elevator,
      ManipulatorPosition position) {
    return createPositionCommand(manipulatorState, diffArm, elevator, position, false);
  }

  /**
   * Creates a command that drives the elevator and differential arm to the supplied setpoints and
   * marks the manipulator state machine as ready once the position is reached.
   */
  public static Command createScoreCommand( // final pre score pos command
      ManipulatorStateMachine manipulatorState,
      DifferentialSubsystem diffArm,
      ElevatorSubsystem elevator,
      ManipulatorPosition position) {
    return createPositionCommand(manipulatorState, diffArm, elevator, position, true);
  }

  /**
   * Creates a command that drives the manipulator to the supplied setpoints sequentially in a safe
   * deploy order (extension, rotation, then elevator) without marking the manipulator as ready when
   * finished.
   */
  public static Command createSafeDeployCommand( // sequential pos command
      ManipulatorStateMachine manipulatorState,
      DifferentialSubsystem diffArm,
      ElevatorSubsystem elevator,
      ManipulatorPosition position) {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            Commands.runOnce(
                () -> {
                  // Reset state machine flags so the new goal can be evaluated cleanly.
                  manipulatorState.atGoalState(false);
                  manipulatorState.failedToReachGoal(false);
                }),
            diffArm.setExtensionSetpointCommand(position.extensionMillimeters()),
            diffArm.setRotationSetpointCommand(position.rotationDegrees()),
            elevator.setElevatorSetpointCommand(position.elevatorMeters()));

    command.addRequirements(diffArm, elevator);
    return command;
  }

  /**
   * Creates a command that drives the manipulator to the supplied setpoints sequentially in a safe
   * return order (elevator, rotation, then extension) without marking the manipulator as ready when
   * finished.
   */
  public static Command createSafeReturnCommand(
      ManipulatorStateMachine manipulatorState,
      DifferentialSubsystem diffArm,
      ElevatorSubsystem elevator,
      ManipulatorPosition position) {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            Commands.runOnce(
                () -> {
                  // Reset state machine flags so the new goal can be evaluated cleanly.
                  manipulatorState.atGoalState(false);
                  manipulatorState.failedToReachGoal(false);
                }),
            elevator.setElevatorSetpointCommand(position.elevatorMeters()),
            diffArm.setRotationSetpointCommand(position.rotationDegrees()),
            diffArm.setExtensionSetpointCommand(position.extensionMillimeters()));

    command.addRequirements(diffArm, elevator);
    return command;
  }

  private static Command createPositionCommand(
      ManipulatorStateMachine manipulatorState,
      DifferentialSubsystem diffArm,
      ElevatorSubsystem elevator,
      ManipulatorPosition position,
      boolean markReadyOnFinish) {
    SequentialCommandGroup command =
        new SequentialCommandGroup(
            Commands.runOnce(
                () -> {
                  // Reset state machine flags so the new goal can be evaluated cleanly.
                  manipulatorState.atGoalState(false);
                  manipulatorState.failedToReachGoal(false);
                }),
            new ParallelCommandGroup(
                diffArm.setRotAndExtSetpointCommand(
                    position.extensionMillimeters(), position.rotationDegrees()),
                elevator.setElevatorSetpointCommand(position.elevatorMeters())));

    if (markReadyOnFinish) {
      command.addCommands(Commands.runOnce(() -> manipulatorState.atGoalState(true)));
    }

    command.addRequirements(diffArm, elevator);
    return command;
  }

  // private static Command createPositionCommand(
  //     ManipulatorStateMachine manipulatorState,
  //     DifferentialSubsystem diffArm,
  //     ElevatorSubsystem elevator,
  //     ManipulatorPosition position,
  //     boolean markReadyOnFinish) {

  //   if (markReadyOnFinish) {
  //     return Commands.sequence(
  //         // initialize (requirements passed here)
  //         Commands.runOnce(
  //             () -> {
  //               manipulatorState.atGoalState(false);
  //               manipulatorState.failedToReachGoal(false);
  //               System.out.println("L3Prep Initializing");
  //               diffArm.setExtensionSetpoint(position.extensionMillimeters());
  //               diffArm.setRotationSetpoint(position.rotationDegrees());
  //               elevator.setElevatorSetpoint(position.elevatorMeters());
  //             },
  //             diffArm,
  //             elevator),

  //         // wait until at setpoints (runs every scheduler cycle and finishes when condition
  // true)
  //         Commands.run(
  //                 () -> {
  //                   // optional: things to do each loop (logging, telemetry, etc.)
  //                   if (diffArm.atRotationSetpoint()
  //                       && elevator.atPosition()
  //                       && diffArm.atExtenstionSetpoint()) {
  //                     System.out.println("L3Prep Done");
  //                   }
  //                 },
  //                 diffArm,
  //                 elevator)
  //             .until(
  //                 () ->
  //                     diffArm.atRotationSetpoint()
  //                         && elevator.atPosition()
  //                         && diffArm.atExtenstionSetpoint()),

  //         // mark ready at the end
  //         Commands.runOnce(() -> manipulatorState.atGoalState(true), diffArm, elevator));
  //   } else {
  //     return Commands.sequence(
  //         Commands.runOnce(
  //             () -> {
  //               manipulatorState.atGoalState(false);
  //               manipulatorState.failedToReachGoal(false);
  //               System.out.println("L3Prep Initializing");
  //               diffArm.setExtensionSetpoint(position.extensionMillimeters());
  //               diffArm.setRotationSetpoint(position.rotationDegrees());
  //               elevator.setElevatorSetpoint(position.elevatorMeters());
  //             },
  //             diffArm,
  //             elevator),
  //         Commands.run(
  //                 () -> {
  //                   if (diffArm.atRotationSetpoint()
  //                       && elevator.atPosition()
  //                       && diffArm.atExtenstionSetpoint()) {
  //                     System.out.println("L3Prep Done");
  //                   }
  //                 },
  //                 diffArm,
  //                 elevator)
  //             .until(
  //                 () ->
  //                     diffArm.atRotationSetpoint()
  //                         && elevator.atPosition()
  //                         && diffArm.atExtenstionSetpoint()));
  //   }
  // }
}
