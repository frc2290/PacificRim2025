package frc.robot.commands;

import static org.mockito.Mockito.*;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class IntakeCoralCommandTest {

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
  void intakeCoralStopsAfterPieceLeaves() {
    when(manipulator.seesCoral()).thenReturn(true, false);

    IntakeCoral cmd = new IntakeCoral(manipulator, state);
    cmd.initialize();
    cmd.execute();
    verify(manipulator).intake(0.55);

    when(manipulator.seesCoral()).thenReturn(false);
    cmd.execute();

    cmd.end(false);
    verify(manipulator, times(2)).intake(0.55);
    verify(manipulator).intake(0);
    verify(manipulator).setCoral(true);
  }
}
