package frc.robot.commands.Waits;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import frc.robot.subsystems.DifferentialSubsystem;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class ExtensionSetWaitTest {

  @Mock private DifferentialSubsystem diff;

  @Test
  void waitsForExtensionSetpoint() {
    when(diff.atExtenstionSetpoint()).thenReturn(false, true);
    ExtensionSetWait cmd = new ExtensionSetWait(diff, 7.0);

    cmd.initialize();
    verify(diff).setExtensionSetpoint(7.0);

    assertFalse(cmd.isFinished());
    assertTrue(cmd.isFinished());
  }
}
