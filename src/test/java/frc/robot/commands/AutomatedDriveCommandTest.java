package frc.robot.commands;

import static org.mockito.Mockito.*;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.DriveState;
import frc.utils.PoseEstimatorSubsystem;

@ExtendWith(MockitoExtension.class)
class AutomatedDriveCommandTest {

    @Mock private StateSubsystem state;
    @Mock private DriveSubsystem drive;
    @Mock private DifferentialSubsystem diff;
    @Mock private PoseEstimatorSubsystem pose;
    @Mock private XboxController controller;

    @BeforeEach
    void setup() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        CommandScheduler.getInstance().enable();

        when(drive.getRotPidController()).thenReturn(new PIDController(0, 0, 0));
        when(drive.getXPidController()).thenReturn(new PIDController(0, 0, 0));
        when(drive.getYPidController()).thenReturn(new PIDController(0, 0, 0));
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    @Test
    void usesControllerInputsWhenRotating() {
        when(controller.getLeftY()).thenReturn(0.2);
        when(controller.getLeftX()).thenReturn(0.1);
        when(controller.getRightX()).thenReturn(0.3);

        AutomatedDrive cmd = new AutomatedDrive(state, drive, diff, pose, controller);
        cmd.initialize();
        cmd.execute();

        var xCaptor = org.mockito.ArgumentCaptor.forClass(Double.class);
        var yCaptor = org.mockito.ArgumentCaptor.forClass(Double.class);
        var rCaptor = org.mockito.ArgumentCaptor.forClass(Double.class);
        verify(drive).drive(xCaptor.capture(), yCaptor.capture(), rCaptor.capture(), eq(true));

        double expectedX = -edu.wpi.first.math.MathUtil.applyDeadband(0.2, frc.robot.Constants.OIConstants.kDriveDeadband);
        double expectedY = -edu.wpi.first.math.MathUtil.applyDeadband(0.1, frc.robot.Constants.OIConstants.kDriveDeadband);
        double expectedR = -edu.wpi.first.math.MathUtil.applyDeadband(0.3, frc.robot.Constants.OIConstants.kDriveDeadband);
        assertEquals(expectedX, xCaptor.getValue(), 1e-9);
        assertEquals(expectedY, yCaptor.getValue(), 1e-9);
        assertEquals(expectedR, rCaptor.getValue(), 1e-9);
    }
}

