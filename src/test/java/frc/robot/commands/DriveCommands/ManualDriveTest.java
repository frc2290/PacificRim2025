package frc.robot.commands.DriveCommands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.eq;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveStateMachine;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PoseEstimatorSubsystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class ManualDriveTest {

  @Mock private DriveStateMachine stateMachine;
  @Mock private DriveSubsystem drive;
  @Mock private PoseEstimatorSubsystem pose;
  @Mock private XboxController controller;

  @BeforeEach
  void setup() {
    when(drive.getRotPidController()).thenReturn(new PIDController(0, 0, 0));
    when(drive.getXPidController()).thenReturn(new PIDController(0, 0, 0));
    when(drive.getYPidController()).thenReturn(new PIDController(0, 0, 0));
  }

  @Test
  void passesControllerInputsToDriveSubsystem() {
    when(controller.getLeftY()).thenReturn(0.2);
    when(controller.getLeftX()).thenReturn(-0.3);
    when(controller.getRightX()).thenReturn(0.4);

    ManualDrive command = new ManualDrive(stateMachine, drive, pose, controller);
    command.execute();

    ArgumentCaptor<Double> xCaptor = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> yCaptor = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> rotCaptor = ArgumentCaptor.forClass(Double.class);

    verify(drive).drive(xCaptor.capture(), yCaptor.capture(), rotCaptor.capture(), eq(true));

    double expectedX = -MathUtil.applyDeadband(0.2, OIConstants.kDriveDeadband);
    double expectedY = -MathUtil.applyDeadband(-0.3, OIConstants.kDriveDeadband);
    double expectedRot = -MathUtil.applyDeadband(0.4, OIConstants.kDriveDeadband);

    assertEquals(expectedX, xCaptor.getValue(), 1e-9);
    assertEquals(expectedY, yCaptor.getValue(), 1e-9);
    assertEquals(expectedRot, rotCaptor.getValue(), 1e-9);
  }
}
