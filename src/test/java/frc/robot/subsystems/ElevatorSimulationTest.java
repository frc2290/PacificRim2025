package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import com.revrobotics.sim.SparkRelativeEncoderSim;

public class ElevatorSimulationTest {
    @Test
    public void simulatedEncoderFeedsPosition() throws Exception {
        assertTrue(HAL.initialize(500, 0));

        ElevatorSubsystem elevator = new ElevatorSubsystem();

        Field simField = ElevatorSubsystem.class.getDeclaredField("leftEncoderSim");
        simField.setAccessible(true);
        SparkRelativeEncoderSim encSim = (SparkRelativeEncoderSim) simField.get(elevator);

        encSim.setPosition(0.4);
        assertEquals(0.4, elevator.getPosition(), 1e-5);
    }
}
