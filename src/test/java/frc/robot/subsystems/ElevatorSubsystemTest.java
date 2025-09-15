package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class ElevatorSubsystemTest {

  @Mock private SparkFlex leftMotor;
  @Mock private SparkFlex rightMotor;
  @Mock private RelativeEncoder encoder;
  @Mock private SparkClosedLoopController controller;

  private ElevatorSubsystem elevator;

  @BeforeAll
  static void setupHal() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().enable();
    elevator = new ElevatorSubsystem(leftMotor, rightMotor, encoder, controller);
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  void setElevatorSetpointStoresValue() {
    elevator.setElevatorSetpoint(1.0);
    assertEquals(1.0, elevator.getElevatorSetpoint(), 1e-9);
  }

  @Test
  void setElevatorSetpointAcceptsNegative() {
    elevator.setElevatorSetpoint(-0.5);
    assertEquals(-0.5, elevator.getElevatorSetpoint(), 1e-9);
  }

  @Test
  void setElevatorSetpointCommandUpdatesSetpointOnInit() {
    Command cmd = elevator.setElevatorSetpointCommand(0.75);
    cmd.initialize();
    assertEquals(0.75, elevator.getElevatorSetpoint(), 1e-9);
  }

  @Test
  void incrementElevatorSetpointAddsIncrement() {
    elevator.setElevatorSetpoint(0.2);
    Command cmd = elevator.incrementElevatorSetpoint(0.3);
    cmd.initialize();
    assertEquals(0.5, elevator.getElevatorSetpoint(), 1e-9);
  }

  @Test
  void incrementElevatorSetpointHandlesNegative() {
    elevator.setElevatorSetpoint(0.6);
    elevator.incrementElevatorSetpoint(-0.2).initialize();
    assertEquals(0.4, elevator.getElevatorSetpoint(), 1e-9);
  }

  @Test
  void incrementElevatorSetpointWithZeroLeavesValue() {
    elevator.setElevatorSetpoint(0.3);
    elevator.incrementElevatorSetpoint(0.0).initialize();
    assertEquals(0.3, elevator.getElevatorSetpoint(), 1e-9);
  }

  @Test
  void setSpeedPassesThroughToMotor() {
    elevator.setSpeed(0.8);
    verify(leftMotor).set(0.8);
  }

  @Test
  void setSpeedAcceptsNegativeAndOutOfRange() {
    elevator.setSpeed(-1.2);
    verify(leftMotor).set(-1.2);
  }

  @Test
  void getPositionReturnsEncoderValue() {
    when(encoder.getPosition()).thenReturn(1.23);
    assertEquals(1.23, elevator.getPosition(), 1e-9);
  }

  @Test
  void atPositionWithinToleranceReturnsTrue() {
    elevator.setElevatorSetpoint(2.0);
    when(encoder.getPosition()).thenReturn(2.03);
    assertTrue(elevator.atPosition());
  }

  @Test
  void atPositionOnBoundaryReturnsTrue() {
    elevator.setElevatorSetpoint(2.0);
    when(encoder.getPosition()).thenReturn(1.96);
    assertTrue(elevator.atPosition());
    when(encoder.getPosition()).thenReturn(2.04);
    assertTrue(elevator.atPosition());
  }

  @Test
  void atPositionOutsideToleranceReturnsFalse() {
    elevator.setElevatorSetpoint(2.0);
    when(encoder.getPosition()).thenReturn(2.05);
    assertFalse(elevator.atPosition());
  }

  @Test
  void getCurrentDrawSumsBothMotors() {
    when(leftMotor.getOutputCurrent()).thenReturn(1.5);
    when(rightMotor.getOutputCurrent()).thenReturn(2.0);
    assertEquals(3.5, elevator.getCurrentDraw(), 1e-9);
  }

  @Test
  void periodicCommandsController() {
    when(encoder.getPosition()).thenReturn(0.0);
    elevator.setElevatorSetpoint(0.0);
    elevator.periodic();
    verify(controller).setReference(0.0, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Test
  void simulationPeriodicWithoutSimHardwareDoesNotThrow() {
    assertDoesNotThrow(() -> elevator.simulationPeriodic());
  }

  @Test
  void elevatorSimulationFeedsBack() throws Exception {
    ElevatorSubsystem elevator = new ElevatorSubsystem();

    Field simField = ElevatorSubsystem.class.getDeclaredField("leftEncoderSim");
    simField.setAccessible(true);
    SparkRelativeEncoderSim encSim = (SparkRelativeEncoderSim) simField.get(elevator);

    encSim.setPosition(0.4);
    assertEquals(0.4, elevator.getPosition(), 1e-5);

    Field simMotorField = ElevatorSubsystem.class.getDeclaredField("leftSim");
    simMotorField.setAccessible(true);
    SparkFlexSim sim = (SparkFlexSim) simMotorField.get(elevator);
    for (int i = 0; i < 50; i++) {
      sim.setAppliedOutput(1.0);
      elevator.simulationPeriodic();
      SimHooks.stepTiming(0.02);
    }
    assertTrue(elevator.getPosition() > 0.0);
  }
}
