package frc.robot.commands;

import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.ArgumentCaptor;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.DriveState;
import frc.utils.PoseEstimatorSubsystem;

@ExtendWith(MockitoExtension.class)
class AutomatedDriveTest {

    @Mock private StateSubsystem state;
    @Mock private DriveSubsystem drive;
    @Mock private DifferentialSubsystem diff;
    @Mock private PoseEstimatorSubsystem pose;
    @Mock private XboxController controller;

    private PIDController rotPid = new PIDController(1, 0, 0);
    private PIDController xPid = new PIDController(1, 0, 0);
    private PIDController yPid = new PIDController(1, 0, 0);

    @BeforeEach
    void setup() {
        when(drive.getRotPidController()).thenReturn(rotPid);
        when(drive.getXPidController()).thenReturn(xPid);
        when(drive.getYPidController()).thenReturn(yPid);

        when(controller.getLeftY()).thenReturn(0.0);
        when(controller.getLeftX()).thenReturn(0.0);
        when(controller.getRightX()).thenReturn(0.0);

        when(state.getRotationLock()).thenReturn(true);
        when(state.getDriveState()).thenReturn(DriveState.Teleop);
        when(state.hasCoral()).thenReturn(false);
        when(state.atAlgaePosition()).thenReturn(false);
        when(state.hasAlgae()).thenReturn(false);

        when(pose.getDegrees()).thenReturn(30.0);
    }

    @Test
    void teleopWithoutTargetHoldsHeading() {
        AutomatedDrive cmd = new AutomatedDrive(state, drive, diff, pose, controller);
        cmd.execute();

        ArgumentCaptor<Double> rotCaptor = ArgumentCaptor.forClass(Double.class);
        verify(drive).drive(anyDouble(), anyDouble(), rotCaptor.capture(), eq(true));
        assertEquals(0.0, rotCaptor.getValue(), 1e-9);
    }
}
