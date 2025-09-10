package frc.robot.commands.Waits;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import frc.robot.subsystems.DifferentialSubsystem;

@ExtendWith(MockitoExtension.class)
class ExtensionAndRotationWaitTest {

    @Mock private DifferentialSubsystem diff;

    @Test
    void waitsForBothSetpoints() {
        when(diff.atExtenstionSetpoint()).thenReturn(true);
        when(diff.atRotationSetpoint()).thenReturn(false, true);
        ExtensionAndRotationWait cmd = new ExtensionAndRotationWait(diff, 5.0, 10.0);

        cmd.initialize();
        verify(diff).setExtensionSetpoint(5.0);
        verify(diff).setRotationSetpoint(10.0);

        assertFalse(cmd.isFinished());
        assertTrue(cmd.isFinished());
    }
}

