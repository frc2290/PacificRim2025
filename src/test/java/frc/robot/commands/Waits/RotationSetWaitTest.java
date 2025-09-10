package frc.robot.commands.Waits;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import frc.robot.subsystems.DifferentialSubsystem;

@ExtendWith(MockitoExtension.class)
class RotationSetWaitTest {

    @Mock private DifferentialSubsystem diff;

    @Test
    void waitsForRotationSetpoint() {
        when(diff.atRotationSetpoint()).thenReturn(false, true);
        RotationSetWait cmd = new RotationSetWait(diff, 90.0);

        cmd.initialize();
        verify(diff).setRotationSetpoint(90.0);

        assertFalse(cmd.isFinished());
        assertTrue(cmd.isFinished());
    }
}

