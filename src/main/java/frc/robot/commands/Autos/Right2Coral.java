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

/**
 * Two piece autonomous starting on the right side: scores, loads from the feeder, and scores again.
 */
public class Right2Coral extends SequentialCommandGroup {

    public Right2Coral(PoseEstimatorSubsystem pose,
                       DriveStateMachine driveState,
                       StateMachineCoardinator coordinator,
                       ManipulatorStateMachine manipulatorState,
                       ManipulatorSubsystem manipulator) {
        try {
            PathPlannerPath startToReef = PathPlannerPath.fromPathFile("RightCoral1");
            PathPlannerPath reefToFeeder = PathPlannerPath.fromPathFile("RightCoral1ToFeeder");
            PathPlannerPath feederToReef = PathPlannerPath.fromPathFile("FeederToRightCoral2");
            AutoRoutineFactory routineFactory =
                new AutoRoutineFactory(pose, coordinator, manipulatorState, manipulator);

            addCommands(
                Commands.runOnce(() -> pose.setCurrentPose(startToReef.getStartingHolonomicPose().get())),
                Commands.runOnce(() -> {
                    driveState.setDriveCommand(DriveState.FOLLOW_PATH);
                    coordinator.score(false);
                    coordinator.setRobotGoal(RobotState.SAFE_CORAL_TRAVEL);
                }),
                routineFactory.scoreCoral(startToReef, RobotState.L4),
                routineFactory.intakeCoral(reefToFeeder),
                routineFactory.scoreCoral(feederToReef, RobotState.L4),
                Commands.runOnce(() -> driveState.setDriveCommand(DriveState.CANCELLED))
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to build Right2Coral auto: " + ex.getMessage(), ex.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
