package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.DriveState;
import frc.robot.subsystems.StateSubsystem.PositionState;

@ExtendWith(MockitoExtension.class)
class AlgaeRemovalL3CommandTest {

    @Mock private ManipulatorSubsystem manipulator;
    @Mock private StateSubsystem state;

    @Test
    void initializesAndCleansUpProperly() {
        AlgaeRemovalL3 cmd = new AlgaeRemovalL3(manipulator, state, false);
        cmd.initialize();
        verify(state).setGoal(PositionState.AlgaeL3Position);
        verify(state).setDriveState(DriveState.Teleop);

        cmd.execute();
        verify(manipulator).intake(-1.0);

        cmd.end(false);
        verify(manipulator).intake(0);
        verify(state).setGoal(PositionState.IntakePosition);
        verify(state).setDriveState(DriveState.CoralStation);
        assertFalse(cmd.isFinished());
    }
}

