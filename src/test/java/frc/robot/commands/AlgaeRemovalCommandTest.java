package frc.robot.commands;

import static org.mockito.Mockito.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.DriveState;
import frc.robot.subsystems.StateSubsystem.PositionState;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class AlgaeRemovalCommandTest {

  @Mock private ManipulatorSubsystem manipulator;
  @Mock private StateSubsystem state;

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
  void algaeRemovalCommandsManipulatorAndState() {
    AlgaeRemoval cmd = new AlgaeRemoval(manipulator, state, true);
    cmd.initialize();
    verify(state).setGoal(PositionState.AlgaeL2Position);
    verify(state).setDriveState(DriveState.Teleop);

    cmd.execute();
    verify(manipulator).intake(-1.0);

    cmd.end(false);
    verify(manipulator).intake(0);
    verify(state).setGoal(PositionState.IntakePosition);
    verify(state).setDriveState(DriveState.CoralStation);
  }
}
