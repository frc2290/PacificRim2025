package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Waits.SetGoalWait;
import frc.robot.subsystems.StateSubsystem.DriveState;
import frc.robot.subsystems.StateSubsystem.PositionState;
import frc.utils.LEDUtility;
import frc.utils.PoseEstimatorSubsystem;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class StateSubsystemTest {

  @Mock private DifferentialSubsystem diff;
  @Mock private ElevatorSubsystem elevator;
  @Mock private DriveSubsystem drive;
  @Mock private ManipulatorSubsystem manipulator;
  @Mock private PoseEstimatorSubsystem pose;
  @Mock private LEDUtility ledUtility;

  private StateSubsystem state;

  @BeforeEach
  void setUp() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().enable();
    state = new StateSubsystem(diff, elevator, drive, manipulator, pose, ledUtility);
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  void atCurrentStateTrueWhenAllSubsystemsReady() {
    when(elevator.atPosition()).thenReturn(true);
    when(diff.atExtenstionSetpoint()).thenReturn(true);
    when(diff.atRotationSetpoint()).thenReturn(true);
    assertTrue(state.atCurrentState());
  }

  @Test
  void atCurrentStateFalseWhenTransitioning() throws Exception {
    when(elevator.atPosition()).thenReturn(true);
    when(diff.atExtenstionSetpoint()).thenReturn(true);
    when(diff.atRotationSetpoint()).thenReturn(true);
    setTransitioning(true);
    assertFalse(state.atCurrentState());
  }

  @Test
  void atCurrentStateFalseWhenElevatorNotAtPosition() {
    when(elevator.atPosition()).thenReturn(false);
    assertFalse(state.atCurrentState());
  }

  @Test
  void atCurrentStateFalseWhenExtensionNotAtSetpoint() {
    when(elevator.atPosition()).thenReturn(true);
    when(diff.atExtenstionSetpoint()).thenReturn(false);
    assertFalse(state.atCurrentState());
  }

  @Test
  void atCurrentStateFalseWhenRotationNotAtSetpoint() {
    when(elevator.atPosition()).thenReturn(true);
    when(diff.atExtenstionSetpoint()).thenReturn(true);
    when(diff.atRotationSetpoint()).thenReturn(false);
    assertFalse(state.atCurrentState());
  }

  @Test
  void setCurrentStateUpdatesPrevAndClearsTransitioning() throws Exception {
    setTransitioning(true);
    state.setCurrentState(PositionState.L1Position);
    assertEquals(PositionState.L1Position, state.getCurrentState());
    assertEquals(PositionState.StartPosition, state.getPrevState());
    assertFalse(state.isTransitioning());
  }

  @Test
  void setGoalUpdatesGoalAndClearsTransitioning() throws Exception {
    setTransitioning(true);
    state.setGoal(PositionState.L2Position);
    assertEquals(PositionState.L2Position, state.getGoalState());
    assertFalse(state.isTransitioning());
  }

  @Test
  void atGoalReturnsTrueOnlyWhenGoalMatchesCurrent() {
    state.setGoal(PositionState.L3Position);
    state.setCurrentState(PositionState.L3Position);
    assertTrue(state.atGoal());
    state.setGoal(PositionState.L2Position);
    assertFalse(state.atGoal());
  }

  @Test
  void cancelCurrentCommandResetsStatesAndSetpoints() {
    when(elevator.getPosition()).thenReturn(3.0);
    when(diff.getExtensionPosition()).thenReturn(4.0);
    when(diff.getRotationPosition()).thenReturn(5.0);

    state.cancelCurrentCommand();

    assertEquals(PositionState.Cancelled, state.getCurrentState());
    assertEquals(PositionState.Cancelled, state.getGoalState());
    verify(elevator).setElevatorSetpoint(3.0);
    verify(diff).setExtensionSetpoint(4.0);
    verify(diff).setRotationSetpoint(5.0);
  }

  @Test
  void cancelCurrentCommandCancelsRunningCommand() throws Exception {
    Command cmd = mock(Command.class);
    setCurrentCommand(cmd);
    state.cancelCurrentCommand();
    verify(cmd).cancel();
    assertNull(getCurrentCommand());
  }

  @Test
  void safeToMoveUsesElevatorThreshold() {
    when(elevator.getPosition()).thenReturn(0.9);
    assertTrue(state.safeToMove());
    when(elevator.getPosition()).thenReturn(1.0);
    assertFalse(state.safeToMove());
  }

  @Test
  void atSafeStateRecognizesSafeStates() {
    assertTrue(state.atSafeState());
    state.setCurrentState(PositionState.IntakePosition);
    assertTrue(state.atSafeState());
    state.setCurrentState(PositionState.Cancelled);
    assertTrue(state.atSafeState());
    state.setCurrentState(PositionState.L1Position);
    assertFalse(state.atSafeState());
  }

  @Test
  void atAlgaePositionTrueForAlgaeStates() {
    state.setCurrentState(PositionState.AlgaeL2Position);
    assertTrue(state.atAlgaePosition());
    state.setCurrentState(PositionState.L1Position);
    assertFalse(state.atAlgaePosition());
    state.setCurrentState(PositionState.AlgaeL3Position);
    assertTrue(state.atAlgaePosition());
  }

  @Test
  void atAlgaePositionTriggerReflectsState() {
    state.setCurrentState(PositionState.AlgaeL3Position);
    assertTrue(state.atAlgaePositionTrigger().getAsBoolean());
    state.setCurrentState(PositionState.L1Position);
    assertFalse(state.atAlgaePositionTrigger().getAsBoolean());
  }

  @Test
  void rightScoreSettersAndCommandWork() {
    assertFalse(state.getRightScore());
    state.setRightScore(true);
    assertTrue(state.getRightScore());
    Command cmd = state.setRightScoreCommand(false);
    cmd.initialize();
    assertFalse(state.getRightScore());
  }

  @Test
  void manipulatorDelegatesForPieceDetection() {
    when(manipulator.hasCoral()).thenReturn(false);
    when(manipulator.hasAlgae()).thenReturn(false);
    assertFalse(state.hasCoral());
    assertFalse(state.hasAlgae());
    when(manipulator.hasCoral()).thenReturn(true);
    when(manipulator.hasAlgae()).thenReturn(true);
    assertTrue(state.hasCoral());
    assertTrue(state.hasAlgae());
  }

  @Test
  void rotationLockControlsAndToggleCommand() {
    assertTrue(state.getRotationLock());
    state.setRotationLock(false);
    assertFalse(state.getRotationLock());
    Command cmd = state.toggleRotationLock();
    cmd.initialize();
    assertTrue(state.getRotationLock());
  }

  @Test
  void driveStateSettersAndCommandWork() {
    assertEquals(DriveState.Teleop, state.getDriveState());
    state.setDriveState(DriveState.Auto);
    assertEquals(DriveState.Auto, state.getDriveState());
    Command cmd = state.setDriveStateCommand(DriveState.ReefScore);
    cmd.initialize();
    assertEquals(DriveState.ReefScore, state.getDriveState());
  }

  @Test
  void setCurrentStateCommandSetsState() {
    Command cmd = state.setCurrentStateCommand(PositionState.L2Position);
    cmd.initialize();
    assertEquals(PositionState.L2Position, state.getCurrentState());
  }

  @Test
  void setGoalCommandSetsGoal() {
    Command cmd = state.setGoalCommand(PositionState.L3Position);
    cmd.initialize();
    assertEquals(PositionState.L3Position, state.getGoalState());
  }

  @Test
  void setGoalCommandWithWaitReturnsWaitCommand() {
    Command cmd = state.setGoalCommand(PositionState.L1Position, true);
    assertTrue(cmd instanceof SetGoalWait);
  }

  @Test
  void autoAndDisabledFlagsAndTriggers() {
    assertFalse(state.isAuto());
    assertFalse(state.isDisabled());
    assertFalse(state.isAutoTrigger().getAsBoolean());
    state.setAuto(true);
    state.setDisabled(true);
    assertTrue(state.isAuto());
    assertTrue(state.isDisabled());
    assertTrue(state.isAutoTrigger().getAsBoolean());
  }

  @Test
  void atTargetTriggerRequiresPoseAndState() {
    when(elevator.atPosition()).thenReturn(true);
    when(diff.atExtenstionSetpoint()).thenReturn(true);
    when(diff.atRotationSetpoint()).thenReturn(true);
    when(pose.atTargetPose()).thenReturn(true);
    assertTrue(state.atTarget().getAsBoolean());
    when(pose.atTargetPose()).thenReturn(false);
    assertFalse(state.atTarget().getAsBoolean());
  }

  @Test
  void cancelCommandRunsCancellation() {
    when(elevator.getPosition()).thenReturn(1.0);
    when(diff.getExtensionPosition()).thenReturn(2.0);
    when(diff.getRotationPosition()).thenReturn(3.0);
    Command cmd = state.cancelCommand();
    cmd.initialize();
    assertEquals(PositionState.Cancelled, state.getCurrentState());
    verify(elevator).setElevatorSetpoint(1.0);
    verify(diff).setExtensionSetpoint(2.0);
    verify(diff).setRotationSetpoint(3.0);
  }

  private void setTransitioning(boolean value) throws Exception {
    Field f = StateSubsystem.class.getDeclaredField("transitioning");
    f.setAccessible(true);
    f.set(state, value);
  }

  private void setCurrentCommand(Command cmd) throws Exception {
    Field f = StateSubsystem.class.getDeclaredField("currentCommand");
    f.setAccessible(true);
    f.set(state, cmd);
  }

  private Command getCurrentCommand() throws Exception {
    Field f = StateSubsystem.class.getDeclaredField("currentCommand");
    f.setAccessible(true);
    return (Command) f.get(state);
  }
}
