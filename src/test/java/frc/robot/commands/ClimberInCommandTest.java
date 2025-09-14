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
import frc.robot.subsystems.DriveSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class ClimberInCommandTest {

  @Mock private SparkMax motor;
  @Mock private SparkClosedLoopController controller;
  @Mock private RelativeEncoder encoder;
  @Mock private Servo servo;
  @Mock private DriveSubsystem drive;

  private ClimbSubsystem climb;

  @BeforeEach
  void setup() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().enable();
    climb = spy(new ClimbSubsystem(motor, controller, encoder, servo));
    climb.climbDistance = 1;
    climb.tClimb = 1;
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  void climberInDrivesAndStopsWhenAtSetpoint() {
    ClimberIn cmd = new ClimberIn(climb, drive);
    cmd.initialize();
    cmd.execute();
    verify(drive).drive(-1.0, 0.0, 0.0, false);
    verify(climb).setClimberSetpoint(Climber.climberInSetpoint);

    cmd.end(false);
    verify(servo).setAngle(180);
    verify(motor).stopMotor();
    assertTrue(climb.getClimbing());
    verify(drive).drive(0.0, 0.0, 0.0, false);
  }
}
