package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.Test;

public class ManipulatorSimulationTest {
  @Test
  public void limitSwitchSequenceToggles() {
    assertTrue(HAL.initialize(500, 0));

    ManipulatorSubsystem manip = new ManipulatorSubsystem();

    assertTrue(manip.hasCoral());
    manip.setCoral(false);
    manip.coralDebounce = new Debouncer(0.0);

    manip.intake(1.0);
    manip.simulationPeriodic();
    assertTrue(manip.seesCoral());

    SimHooks.stepTiming(0.11);
    manip.simulationPeriodic();

    assertFalse(manip.seesCoral());
    assertTrue(manip.hasCoral());
  }
}
