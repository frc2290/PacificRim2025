package frc.utils;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.trajectory.ExponentialProfile;
import java.util.function.Supplier;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ExponentialProfiledPIDControllerTest {
  private double time;
  private Supplier<Double> timeSupplier;
  private ExponentialProfiledPIDController controller;

  @BeforeEach
  void setUp() {
    time = 0.0;
    timeSupplier = () -> time;
    controller =
        new ExponentialProfiledPIDController(
            1.0,
            0.0,
            0.0,
            ExponentialProfile.Constraints.fromCharacteristics(2.0, 1.0, 0.02),
            timeSupplier);
    controller.setTolerance(0.01, 0.05);
  }

  @Test
  void calculateAdvancesSetpoint() {
    controller.setGoal(1.0);
    controller.calculate(0.0);
    time = 0.5;
    controller.calculate(0.0);
    assertTrue(controller.getSetpointPosition() > 0.0);
    assertTrue(controller.getSetpointPosition() < 1.0);
  }

  @Test
  void atGoalReturnsTrueWhenWithinTolerance() {
    controller.setGoal(1.0);
    time = 5.0;
    controller.calculate(1.0);
    assertTrue(controller.atGoal());
  }

  @Test
  void resetReinitializesSetpoint() {
    controller.setGoal(1.0);
    time = 1.0;
    controller.calculate(0.2);
    controller.reset(0.5);
    assertEquals(0.5, controller.getSetpointPosition(), 1e-9);
  }
}
