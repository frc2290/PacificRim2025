package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.EndEffector.ScoreCoral;
import frc.robot.commands.SwerveAutoStep;
import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorStateMachine.ElevatorManipulatorState;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateMachineCoardinator;
import frc.robot.subsystems.StateMachineCoardinator.RobotState;
import frc.utils.PoseEstimatorSubsystem;

/**
 * Utility that assembles the common manipulator/drive command sequences shared across the autos.
 */
public class AutoRoutineFactory {

    private final PoseEstimatorSubsystem pose;
    private final StateMachineCoardinator coordinator;
    private final ManipulatorStateMachine manipulatorState;
    private final ManipulatorSubsystem manipulator;

    public AutoRoutineFactory(PoseEstimatorSubsystem pose,
                              StateMachineCoardinator coordinator,
                              ManipulatorStateMachine manipulatorState,
                              ManipulatorSubsystem manipulator) {
        this.pose = pose;
        this.coordinator = coordinator;
        this.manipulatorState = manipulatorState;
        this.manipulator = manipulator;
    }

    /**
     * Builds the repeated scoring sequence used by the one-, two-, and three-piece routines.
     */
    public Command scoreCoral(PathPlannerPath path, RobotState targetState) {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> {
                manipulatorState.atGoalState(false);
                coordinator.setRobotGoal(targetState);
            }),
            Commands.parallel(new SwerveAutoStep(path, pose), manipulatorState.waitUntilReady()),
            Commands.runOnce(() -> coordinator.score(true)),
            new ScoreCoral(manipulatorState, manipulator),
            Commands.runOnce(() -> {
                coordinator.score(false);
                coordinator.setRobotGoal(RobotState.SAFE_CORAL_TRAVEL);
            }),
            manipulatorState.waitForState(ElevatorManipulatorState.SAFE_CORAL_TRAVEL)
        );
    }

    /**
     * Builds the feeder intake sequence shared by the multi-piece routines.
     */
    public Command intakeCoral(PathPlannerPath path) {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> {
                manipulatorState.atGoalState(false);
                coordinator.score(false);
                coordinator.setRobotGoal(RobotState.INTAKE_CORAL);
            }),
            Commands.parallel(new SwerveAutoStep(path, pose), Commands.waitUntil(() -> manipulator.hasCoral())),
            manipulatorState.waitForState(ElevatorManipulatorState.INTAKE_CORAL),
            Commands.runOnce(() -> coordinator.setRobotGoal(RobotState.SAFE_CORAL_TRAVEL)),
            manipulatorState.waitForState(ElevatorManipulatorState.SAFE_CORAL_TRAVEL)
        );
    }
}
