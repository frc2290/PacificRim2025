package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;

public class ElevatorSimulationTest {
  @Test
  public void elevatorSimulationFeedsBack() throws Exception {
    assertTrue(HAL.initialize(500, 0));

    ElevatorSubsystem elevator = new ElevatorSubsystem();

    Field simField = ElevatorSubsystem.class.getDeclaredField("leftEncoderSim");
    simField.setAccessible(true);
    SparkRelativeEncoderSim encSim = (SparkRelativeEncoderSim) simField.get(elevator);

    encSim.setPosition(0.4);
    assertEquals(0.4, elevator.getPosition(), 1e-5);

    Field simMotorField = ElevatorSubsystem.class.getDeclaredField("leftSim");
    simMotorField.setAccessible(true);
    com.revrobotics.sim.SparkFlexSim sim =
        (com.revrobotics.sim.SparkFlexSim) simMotorField.get(elevator);
    for (int i = 0; i < 50; i++) {
      sim.setAppliedOutput(1.0);
      elevator.simulationPeriodic();
      SimHooks.stepTiming(0.02);
    }
    assertTrue(elevator.getPosition() > 0.0);
  }
}
