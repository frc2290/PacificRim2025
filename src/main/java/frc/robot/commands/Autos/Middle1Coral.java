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

  public Middle1Coral(
      PoseEstimatorSubsystem pose,
      DriveStateMachine driveState,
      StateMachineCoardinator coordinator,
      ManipulatorStateMachine manipulatorState,
      ManipulatorSubsystem manipulator) {
    try {
      PathPlannerPath startToReef = PathPlannerPath.fromPathFile("MiddleCoral1");
      AutoRoutineFactory routineFactory =
          new AutoRoutineFactory(pose, coordinator, manipulatorState, manipulator);

      addCommands(
          // Reset odometry so the robot knows where it is before moving.
          Commands.runOnce(() -> pose.setCurrentPose(startToReef.getStartingHolonomicPose().get())),
          Commands.runOnce(
              () -> {
                // Start path following with the manipulator stowed for travel.
                driveState.setDriveCommand(DriveState.FOLLOW_PATH);
                coordinator.requestToScore(false);
                coordinator.setRobotGoal(RobotState.SAFE_CORAL_TRAVEL);
              }),
          // Drive up to the reef and perform the scoring sequence.
          routineFactory.scoreCoral(startToReef, RobotState.L4),
          // Hand manual control back to the driver afterward.
          Commands.runOnce(() -> driveState.setDriveCommand(DriveState.CANCELLED)));
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to build Middle1Coral auto: " + ex.getMessage(), ex.getStackTrace());
      addCommands(Commands.none());
    }
  }
}
