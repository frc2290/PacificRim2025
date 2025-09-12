package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;

import org.junit.jupiter.api.Test;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;

public class ClimberSimulationTest {
  @Test
  public void simSelectsMassBasedOnDirection() throws Exception {
    HAL.initialize(500, 0);

    SimHooks.restartTiming();
    ClimbSubsystem climb = new ClimbSubsystem();
    climb.setServoOpen();
    Field simField = ClimbSubsystem.class.getDeclaredField("climberSim");
    simField.setAccessible(true);
    SparkMaxSim sim = (SparkMaxSim) simField.get(climb);
    Field armField = ClimbSubsystem.class.getDeclaredField("armSim");
    armField.setAccessible(true);
    Field deployField = ClimbSubsystem.class.getDeclaredField("deploySim");
    deployField.setAccessible(true);
    Field retractField = ClimbSubsystem.class.getDeclaredField("retractSim");
    retractField.setAccessible(true);

    sim.setAppliedOutput(0.5);
    climb.simulationPeriodic();
    assertSame(deployField.get(climb), armField.get(climb));

    sim.setAppliedOutput(-0.5);
    climb.simulationPeriodic();
    assertSame(retractField.get(climb), armField.get(climb));

    Field servoField = ClimbSubsystem.class.getDeclaredField("servo");
    servoField.setAccessible(true);
    ((edu.wpi.first.wpilibj.Servo) servoField.get(climb)).close();
    Field motorField = ClimbSubsystem.class.getDeclaredField("leftMotor");
    motorField.setAccessible(true);
    ((com.revrobotics.spark.SparkMax) motorField.get(climb)).close();
  }

  @Test
  public void ratchetBlocksDeploy() throws Exception {
    HAL.initialize(500, 0);

    SimHooks.restartTiming();
    ClimbSubsystem climb = new ClimbSubsystem();
    Field f = ClimbSubsystem.class.getDeclaredField("climberSim");
    f.setAccessible(true);
    SparkMaxSim sim = (SparkMaxSim) f.get(climb);

    climb.setServoClose();
    sim.setAppliedOutput(1.0);
    for (int i = 0; i < 50; i++) {
      climb.simulationPeriodic();
      SimHooks.stepTiming(0.02);
    }
    assertEquals(0.0, climb.getClimberPos(), 1e-5);

    sim.setAppliedOutput(-1.0);
    for (int i = 0; i < 50; i++) {
      climb.simulationPeriodic();
      SimHooks.stepTiming(0.02);
    }
    assertTrue(Math.abs(climb.getClimberPos()) > 0.0);

    Field servoField = ClimbSubsystem.class.getDeclaredField("servo");
    servoField.setAccessible(true);
    ((edu.wpi.first.wpilibj.Servo) servoField.get(climb)).close();
    Field motorField = ClimbSubsystem.class.getDeclaredField("leftMotor");
    motorField.setAccessible(true);
    ((com.revrobotics.spark.SparkMax) motorField.get(climb)).close();
  }
}
