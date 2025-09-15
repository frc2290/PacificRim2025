package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class ManipulatorSubsystemTest {

  @Mock private SparkFlex motor;
  @Mock private SparkAbsoluteEncoder absEncoder;
  @Mock private RelativeEncoder relEncoder;
  @Mock private SparkLimitSwitch limitSwitch;

  private ManipulatorSubsystem manipulator;

  @BeforeEach
  void setUp() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().enable();
    manipulator =
        new ManipulatorSubsystem(motor, absEncoder, relEncoder, limitSwitch, new Debouncer(0.0));
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  void intakeSetsMotor() {
    manipulator.intake(0.6);
    verify(motor).set(0.6);
  }

  @Test
  void runIntakeCommandSetsMotor() {
    Command cmd = manipulator.runIntake(0.5);
    cmd.initialize();
    verify(motor).set(0.5);
  }

  @Test
  void getWristPosReturnsAbsoluteEncoderValue() {
    when(absEncoder.getPosition()).thenReturn(3.14);
    assertEquals(3.14, manipulator.getWristPos(), 1e-9);
  }

  @Test
  void getMotorPosReadsEncoder() {
    when(relEncoder.getPosition()).thenReturn(2.0);
    assertEquals(2.0, manipulator.getMotorPos(), 1e-9);
  }

  @Test
  void resetMotorPosZerosEncoder() {
    manipulator.resetMotorPos();
    verify(relEncoder).setPosition(0);
  }

  @Test
  void getCurrentDrawReturnsMotorCurrent() {
    when(motor.getOutputCurrent()).thenReturn(5.0);
    assertEquals(5.0, manipulator.getCurrentDraw(), 1e-9);
  }

  @Test
  void coralStateAndTriggerReflectsValue() {
    Trigger t = manipulator.hasCoralTrigger();
    manipulator.setCoral(true);
    assertTrue(manipulator.hasCoral());
    assertTrue(t.getAsBoolean());

    manipulator.setCoral(false);
    assertFalse(manipulator.hasCoral());
    assertFalse(t.getAsBoolean());
  }

  @Test
  void algaeStateAndTriggerReflectsValue() {
    Trigger t = manipulator.hasAlgaeTrigger();
    manipulator.setAlgae(true);
    assertTrue(manipulator.hasAlgae());
    assertTrue(t.getAsBoolean());

    manipulator.setAlgae(false);
    assertFalse(manipulator.hasAlgae());
    assertFalse(t.getAsBoolean());
  }

  @Test
  void seesCoralUsesLimitSwitchWithDebounce() {
    when(limitSwitch.isPressed()).thenReturn(false);
    assertFalse(manipulator.seesCoral());

    when(limitSwitch.isPressed()).thenReturn(true);
    assertTrue(manipulator.seesCoral());
  }

  @Test
  void limitSwitchSequenceToggles() {
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
