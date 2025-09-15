package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class ClimbSubsystemTest {

  @Mock private SparkMax motor;
  @Mock private SparkClosedLoopController controller;
  @Mock private RelativeEncoder encoder;
  @Mock private Servo servo;

  private ClimbSubsystem climb;

  @BeforeEach
  void setUp() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().enable();
    climb = new ClimbSubsystem(motor, controller, encoder, servo);
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  void setClimberSpeedSetsMotor() {
    climb.setClimberSpeed(0.75);
    verify(motor).set(0.75);
  }

  @Test
  void setClimberSpeedCommandRunsMotor() {
    Command cmd = climb.setClimberSpeedCommand(0.5);
    cmd.initialize();
    verify(motor).set(0.5);
  }

  @Test
  void stopClimberMotorStopsMotor() {
    climb.stopClimberMotor();
    verify(motor).stopMotor();
  }

  @Test
  void getClimberPosReturnsEncoderValue() {
    when(encoder.getPosition()).thenReturn(123.0);
    assertEquals(123.0, climb.getClimberPos(), 1e-9);
  }

  @Test
  void getClimberVelReturnsEncoderVelocity() {
    when(encoder.getVelocity()).thenReturn(4.2);
    assertEquals(4.2, climb.getClimberVel(), 1e-9);
  }

  @Test
  void setAndGetClimberSetpointWorks() {
    climb.setClimberSetpoint(42.0);
    assertEquals(42.0, climb.getClimberSetpoint(), 1e-9);
  }

  @Test
  void atClimberSetpointUsesEncoderTolerance() {
    climb.setClimberSetpoint(100.0);
    when(encoder.getPosition()).thenReturn(97.0);
    assertTrue(climb.atClimberSetpoint());
    when(encoder.getPosition()).thenReturn(110.0);
    assertFalse(climb.atClimberSetpoint());
  }

  @Test
  void climberOutChecksRange() {
    when(encoder.getPosition()).thenReturn(-200.0);
    assertTrue(climb.climberOut());
    when(encoder.getPosition()).thenReturn(0.0);
    assertFalse(climb.climberOut());
  }

  @Test
  void getCurrentDrawReturnsMotorCurrent() {
    when(motor.getOutputCurrent()).thenReturn(7.2);
    assertEquals(7.2, climb.getCurrentDraw(), 1e-9);
  }

  @Test
  void periodicCommandsControllerWhenNotClimbing() {
    climb.setClimbing(false);
    climb.periodic();
    verify(controller).setReference(anyDouble(), eq(ControlType.kPosition));
  }

  @Test
  void periodicDoesNotCommandWhenClimbing() {
    climb.setClimbing(true);
    climb.periodic();
    verify(controller, never()).setReference(anyDouble(), any());
  }

  @Test
  void servoControlMethodsWork() {
    climb.setServoOpen();
    verify(servo).setAngle(90);

    climb.setServoClose();
    verify(servo).setAngle(180);

    climb.setServoPos(45);
    verify(servo).setAngle(45);

    when(servo.getPosition()).thenReturn(0.25);
    assertEquals(0.25, climb.getServoPos(), 1e-9);
  }

  @Test
  void ratchetPreventsDeploySpeed() {
    climb.setServoClose();
    climb.setClimberSpeed(-0.5);
    verify(motor).set(0.0);
  }

  @Test
  void simSelectsMassBasedOnDirection() throws Exception {
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
    ((Servo) servoField.get(climb)).close();
    Field motorField = ClimbSubsystem.class.getDeclaredField("leftMotor");
    motorField.setAccessible(true);
    ((SparkMax) motorField.get(climb)).close();
  }

  @Test
  void ratchetBlocksDeploy() throws Exception {
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
    ((Servo) servoField.get(climb)).close();
    Field motorField = ClimbSubsystem.class.getDeclaredField("leftMotor");
    motorField.setAccessible(true);
    ((SparkMax) motorField.get(climb)).close();
  }
}
