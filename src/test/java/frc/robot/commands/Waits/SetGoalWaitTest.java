package frc.robot.commands.Waits;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.PositionState;

@ExtendWith(MockitoExtension.class)
class SetGoalWaitTest {

    @Mock private StateSubsystem state;

    @Test
    void waitsUntilGoalReached() {
        when(state.atGoal()).thenReturn(false, true);
        SetGoalWait cmd = new SetGoalWait(state, PositionState.TravelPosition);

        cmd.initialize();
        verify(state).setGoal(PositionState.TravelPosition);

        assertFalse(cmd.isFinished());
        assertTrue(cmd.isFinished());
    }
}

