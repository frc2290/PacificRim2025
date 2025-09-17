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

/** Simple middle auto that scores a single coral. */
public class Middle1Coral extends SequentialCommandGroup {

    public Middle1Coral(PoseEstimatorSubsystem pose,
                        DriveStateMachine driveState,
                        StateMachineCoardinator coordinator,
                        ManipulatorStateMachine manipulatorState,
                        ManipulatorSubsystem manipulator) {
        try {
            PathPlannerPath startToReef = PathPlannerPath.fromPathFile("MiddleCoral1");
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
                Commands.runOnce(() -> driveState.setDriveCommand(DriveState.CANCELLED))
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to build Middle1Coral auto: " + ex.getMessage(), ex.getStackTrace());
            addCommands(Commands.none());
        }
    }
}
