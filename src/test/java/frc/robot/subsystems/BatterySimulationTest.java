package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * Ensures that subsystem motor simulations draw their bus voltage from the
 * robot controller's battery simulation.
 */
public class BatterySimulationTest {
  @BeforeAll
  public static void setupHAL() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  public void motorsFollowRobotControllerBatteryVoltage() throws Exception {
    // Drive subsystem
    DriveSubsystem drive = new DriveSubsystem();
    verifySubsystem(drive, collectDriveMotors(drive));

    // Climb subsystem
    ClimbSubsystem climb = new ClimbSubsystem();
    verifySubsystem(climb, List.of(getMotor(climb, "leftMotor")));
    closeIfPresent(climb, "servo");

    // Differential arm
    DifferentialSubsystem diff = new DifferentialSubsystem();
    verifySubsystem(diff, List.of(
        getMotor(diff, "leftMotor"),
        getMotor(diff, "rightMotor")));

    // Elevator
    ElevatorSubsystem elevator = new ElevatorSubsystem();
    verifySubsystem(elevator, List.of(
        getMotor(elevator, "leftMotor"),
        getMotor(elevator, "rightMotor")));

    // Manipulator
    ManipulatorSubsystem manip = new ManipulatorSubsystem();
    verifySubsystem(manip, List.of(getMotor(manip, "manipulatorMotor")));
  }

  private void verifySubsystem(Object subsystem, List<SparkBase> motors) {
    // Start from a nonstandard voltage to ensure simulation updates RoboRioSim
    RoboRioSim.setVInVoltage(7.0);
    if (subsystem instanceof edu.wpi.first.wpilibj2.command.SubsystemBase) {
      edu.wpi.first.wpilibj2.command.SubsystemBase s =
          (edu.wpi.first.wpilibj2.command.SubsystemBase) subsystem;
      // First pass uses the pre-set voltage and updates the battery model
      s.simulationPeriodic();
      // Second pass should now use the simulated battery voltage
      s.simulationPeriodic();
    }
    double battery = RobotController.getBatteryVoltage();
    assertTrue(battery > 0.0, "Battery voltage was not set by simulation");
    assertNotEquals(7.0, battery, 1e-5, "Battery voltage did not change");
    for (SparkBase motor : motors) {
      assertEquals(battery, motor.getBusVoltage(), 1e-5);
      motor.close();
    }
  }

  private SparkBase getMotor(Object obj, String field) throws Exception {
    Field f = obj.getClass().getDeclaredField(field);
    f.setAccessible(true);
    return (SparkBase) f.get(obj);
  }

  private List<SparkBase> collectDriveMotors(DriveSubsystem drive) throws Exception {
    Field modulesField = DriveSubsystem.class.getDeclaredField("m_modules");
    modulesField.setAccessible(true);
    Object[] modules = (Object[]) modulesField.get(drive);
    List<SparkBase> motors = new ArrayList<>();
    Field driveField = MAXSwerveModule.class.getDeclaredField("m_drivingSpark");
    driveField.setAccessible(true);
    Field turnField = MAXSwerveModule.class.getDeclaredField("m_turningSpark");
    turnField.setAccessible(true);
    for (Object module : modules) {
      motors.add((SparkBase) driveField.get(module));
      motors.add((SparkBase) turnField.get(module));
    }
    closeIfPresent(drive, "m_gyro");
    return motors;
  }

  private void closeIfPresent(Object obj, String field) {
    try {
      Field f = obj.getClass().getDeclaredField(field);
      f.setAccessible(true);
      Object device = f.get(obj);
      if (device instanceof AutoCloseable) {
        ((AutoCloseable) device).close();
      }
    } catch (NoSuchFieldException | IllegalAccessException ignored) {
      // Field not present or not accessible; ignore.
    } catch (Exception e) {
      // Suppress close exceptions
    }
  }
}
