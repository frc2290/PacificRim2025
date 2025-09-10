package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.PositionState;

@ExtendWith(MockitoExtension.class)
class IntakeAlgaeCommandTest {

    @Mock private ManipulatorSubsystem manipulator;
    @Mock private StateSubsystem state;

    @Test
    void finishesAfterCurrentSpikeAndSetsProcessorGoal() {
        Timer current = new Timer();
        Timer delay = new Timer();
        when(manipulator.getOutputCurrent()).thenReturn(35.0);

        IntakeAlgae cmd = new IntakeAlgae(manipulator, state, current, delay);
        cmd.initialize();

        cmd.execute();
        verify(manipulator).intake(-0.9);

        Timer.delay(1.1);
        cmd.execute();
        Timer.delay(0.6);

        assertTrue(cmd.isFinished());
        cmd.end(false);

        verify(manipulator).intake(-0.5);
        verify(manipulator).setAlgae(true);
        verify(state).setGoal(PositionState.ProcessorPosition);
    }

    @Test
    void interruptionSkipsAlgaeFlag() {
        IntakeAlgae cmd = new IntakeAlgae(manipulator, state, new Timer(), new Timer());
        cmd.end(true);
        verify(manipulator, never()).setAlgae(true);
        verify(state, never()).setGoal(any());
    }
}

