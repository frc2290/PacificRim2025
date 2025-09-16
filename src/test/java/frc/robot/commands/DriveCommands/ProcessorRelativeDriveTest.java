package frc.robot.commands.DriveCommands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.eq;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
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
class ProcessorRelativeDriveTest {

  @Mock private DriveStateMachine stateMachine;
  @Mock private DriveSubsystem drive;
  @Mock private PoseEstimatorSubsystem poseEstimator;
  @Mock private XboxController controller;
  @Mock private PIDController rotPid;
  @Mock private PIDController xPid;
  @Mock private PIDController yPid;

  @BeforeEach
  void setup() {
    when(drive.getRotPidController()).thenReturn(rotPid);
    when(drive.getXPidController()).thenReturn(xPid);
    when(drive.getYPidController()).thenReturn(yPid);
  }

  @Test
  void usesManualRotationWhenDriverCommandsTurn() {
    when(controller.getLeftY()).thenReturn(0.3);
    when(controller.getLeftX()).thenReturn(-0.2);
    when(controller.getRightX()).thenReturn(0.25);

    ProcessorRelativeDrive command =
        new ProcessorRelativeDrive(stateMachine, drive, poseEstimator, controller);
    command.execute();

    double expectedX = -MathUtil.applyDeadband(0.3, OIConstants.kDriveDeadband);
    double expectedY = -MathUtil.applyDeadband(-0.2, OIConstants.kDriveDeadband);
    double expectedRot = -MathUtil.applyDeadband(0.25, OIConstants.kDriveDeadband);

    ArgumentCaptor<Double> xCaptor = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> yCaptor = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> rotCaptor = ArgumentCaptor.forClass(Double.class);

    verify(rotPid, never()).calculate(anyDouble(), anyDouble());
    verify(poseEstimator, never()).turnToTarget(any());
    verify(drive).drive(xCaptor.capture(), yCaptor.capture(), rotCaptor.capture(), eq(true));

    assertEquals(expectedX, xCaptor.getValue(), 1e-9);
    assertEquals(expectedY, yCaptor.getValue(), 1e-9);
    assertEquals(expectedRot, rotCaptor.getValue(), 1e-9);
  }

  @Test
  void alignsToProcessorWhenDriverDoesNotRotate() {
    when(controller.getLeftY()).thenReturn(0.1);
    when(controller.getLeftX()).thenReturn(0.2);
    when(controller.getRightX()).thenReturn(0.0);
    when(poseEstimator.turnToTarget(VisionConstants.processor)).thenReturn(123.0);
    when(poseEstimator.getDegrees()).thenReturn(12.0);
    when(rotPid.calculate(12.0, 123.0)).thenReturn(0.42);

    ProcessorRelativeDrive command =
        new ProcessorRelativeDrive(stateMachine, drive, poseEstimator, controller);
    command.execute();

    double expectedX = -MathUtil.applyDeadband(0.1, OIConstants.kDriveDeadband);
    double expectedY = -MathUtil.applyDeadband(0.2, OIConstants.kDriveDeadband);

    ArgumentCaptor<Double> xCaptor = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> yCaptor = ArgumentCaptor.forClass(Double.class);
    ArgumentCaptor<Double> rotCaptor = ArgumentCaptor.forClass(Double.class);

    verify(poseEstimator).turnToTarget(VisionConstants.processor);
    verify(rotPid).calculate(12.0, 123.0);
    verify(drive).drive(xCaptor.capture(), yCaptor.capture(), rotCaptor.capture(), eq(true));

    assertEquals(expectedX, xCaptor.getValue(), 1e-9);
    assertEquals(expectedY, yCaptor.getValue(), 1e-9);
    assertEquals(0.42, rotCaptor.getValue(), 1e-9);
  }
}
