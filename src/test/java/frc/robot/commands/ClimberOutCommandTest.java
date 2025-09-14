package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.PositionState;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class ClimberOutCommandTest {

  @Mock private SparkMax motor;
  @Mock private SparkClosedLoopController controller;
  @Mock private RelativeEncoder encoder;
  @Mock private Servo servo;
  @Mock private StateSubsystem state;

  private ClimbSubsystem climb;

  @BeforeEach
  void setup() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().enable();
    climb = new ClimbSubsystem(motor, controller, encoder, servo);
    when(state.atGoal()).thenReturn(true);
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  void climberOutSetsGoalAndFinishes() {
    ClimberOut cmd = new ClimberOut(climb, state);
    cmd.initialize();
    cmd.execute();
    verify(state).setGoal(PositionState.ClimbPosition);
    verify(servo).setAngle(90);
    assertFalse(climb.getClimbing());
    assertEquals(Climber.climberOutSetpoint, climb.getClimberSetpoint(), 1e-9);

    cmd.end(false);
  }
}
