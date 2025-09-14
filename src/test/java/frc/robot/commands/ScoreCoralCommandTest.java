package frc.robot.commands;

import static org.mockito.Mockito.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.PositionState;
import frc.utils.PoseEstimatorSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class ScoreCoralCommandTest {

  @Mock private ManipulatorSubsystem manipulator;
  @Mock private DifferentialSubsystem diff;
  @Mock private StateSubsystem state;
  @Mock private PoseEstimatorSubsystem pose;

  @BeforeEach
  void setup() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().enable();
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  void scoreCoralCommandsManipulatorAndState() {
    when(diff.hasLaserCanDistance()).thenReturn(true);
    when(pose.atTargetPose(true)).thenReturn(true);
    when(state.atCurrentState()).thenReturn(true);

    ScoreCoral cmd = new ScoreCoral(manipulator, diff, state, pose, new Timer());
    cmd.initialize();
    cmd.execute();
    verify(manipulator).resetMotorPos();
    verify(manipulator).intake(1);

    cmd.end(false);
    verify(manipulator).intake(0);
    verify(manipulator).setCoral(false);
    verify(manipulator).setAlgae(false);
    verify(state).setGoal(PositionState.IntakePosition);
  }

  @Test
  void endOnInterruptStopsManipulator() {
    ScoreCoral cmd = new ScoreCoral(manipulator, diff, state, pose, new Timer());
    cmd.end(true);
    verify(manipulator).intake(0);
  }
}
