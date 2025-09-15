package frc.robot.io;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Method;
import org.junit.jupiter.api.Test;

/**
 * Uses reflection to ensure real and simulation IO classes expose the same API for transparency
 * between robot and simulator implementations.
 */
class IOParityReflectionTest {

  private static void assertMethodsMatch(Class<?> iface, Class<?> real, Class<?> sim) {
    assertTrue(iface.isAssignableFrom(real));
    assertTrue(iface.isAssignableFrom(sim));

    Method[] ifaceMethods = iface.getMethods();
    for (Method m : ifaceMethods) {
      try {
        Method realMethod = real.getMethod(m.getName(), m.getParameterTypes());
        Method simMethod = sim.getMethod(m.getName(), m.getParameterTypes());
        assertEquals(m.getReturnType(), realMethod.getReturnType());
        assertEquals(m.getReturnType(), simMethod.getReturnType());
      } catch (NoSuchMethodException e) {
        fail("Missing method " + m.getName());
      }
    }
  }

  @Test
  void driveIoParity() {
    assertMethodsMatch(DriveIO.class, DriveIOReal.class, DriveIOSim.class);
  }

  @Test
  void differentialArmIoParity() {
    assertMethodsMatch(
        DifferentialArmIO.class, DifferentialArmIOReal.class, DifferentialArmIOSim.class);
  }

  @Test
  void elevatorIoParity() {
    assertMethodsMatch(ElevatorIO.class, ElevatorIOReal.class, ElevatorIOSim.class);
  }

  @Test
  void manipulatorIoParity() {
    assertMethodsMatch(ManipulatorIO.class, ManipulatorIOReal.class, ManipulatorIOSim.class);
  }

  @Test
  void visionIoParity() {
    assertMethodsMatch(VisionIO.class, VisionIOReal.class, VisionIOSim.class);
  }
}
