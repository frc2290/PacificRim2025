package frc.robot.commands.Waits;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import frc.robot.subsystems.ElevatorSubsystem;

@ExtendWith(MockitoExtension.class)
class ElevatorSetWaitTest {

    @Mock private ElevatorSubsystem elevator;

    @Test
    void waitsUntilElevatorAtPosition() {
        when(elevator.atPosition()).thenReturn(false, true);
        ElevatorSetWait cmd = new ElevatorSetWait(elevator, 5.0);

        cmd.initialize();
        verify(elevator).setElevatorSetpoint(5.0);

        assertFalse(cmd.isFinished());
        assertTrue(cmd.isFinished());
    }
}

