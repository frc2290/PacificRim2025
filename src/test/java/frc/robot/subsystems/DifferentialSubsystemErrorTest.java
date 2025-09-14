package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertThrows;

import edu.wpi.first.hal.HAL;
import java.lang.reflect.Field;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Verifies that DifferentialSubsystem logs runtime exceptions during simulation. */
public class DifferentialSubsystemErrorTest {

  @BeforeAll
  public static void setupHal() {
    Assertions.assertTrue(HAL.initialize(500, 0));
  }

  @Test
  public void reportsSimulationExceptions() throws Exception {
    try (DifferentialSubsystem diff = new DifferentialSubsystem()) {
      diff.close();

      Field encField = DifferentialSubsystem.class.getDeclaredField("leftEncoderSim");
      encField.setAccessible(true);
      encField.set(diff, null);

      assertThrows(RuntimeException.class, () -> diff.updateSimState(0.02));
    }
  }
}
