package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class ElevatorSubsystemTest {

  @Mock private SparkFlex left;
  @Mock private SparkFlex right;
  @Mock private RelativeEncoder encoder;
  @Mock private SparkClosedLoopController controller;

  private ElevatorSubsystem elevator;

  @BeforeEach
  void setUp() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().enable();
    elevator = new ElevatorSubsystem(left, right, encoder, controller);
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  void setSpeedSetsLeftMotor() {
    elevator.setSpeed(0.25);
    verify(left).set(0.25);
  }

  @Test
  void getPositionReturnsEncoderValue() {
    when(encoder.getPosition()).thenReturn(1.2);
    assertEquals(1.2, elevator.getPosition(), 1e-9);
  }

  @Test
  void atPositionChecksTolerance() {
    elevator.setElevatorSetpoint(2.0);
    when(encoder.getPosition()).thenReturn(2.03);
    assertTrue(elevator.atPosition());

    when(encoder.getPosition()).thenReturn(2.1);
    assertFalse(elevator.atPosition());
  }

  @Test
  void incrementSetpointCommandUpdatesValue() {
    elevator.setElevatorSetpoint(1.0);
    Command cmd = elevator.incrementElevatorSetpoint(0.4);
    cmd.initialize();
    assertEquals(1.4, elevator.getElevatorSetpoint(), 1e-9);
  }

  @Test
  void getCurrentDrawSumsMotors() {
    when(left.getOutputCurrent()).thenReturn(2.0);
    when(right.getOutputCurrent()).thenReturn(3.5);
    assertEquals(5.5, elevator.getCurrentDraw(), 1e-9);
  }

  @Test
  void setAndGetElevatorSetpoint() {
    elevator.setElevatorSetpoint(0.8);
    assertEquals(0.8, elevator.getElevatorSetpoint(), 1e-9);
  }
}
