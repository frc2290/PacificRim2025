package frc.robot.commands.Autos;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.DriveState;
import frc.utils.PoseEstimatorSubsystem;
import java.lang.reflect.Field;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

/**
 * Tests for {@link DriveTestAuto} ensuring its completion behavior is consistent between real
 * hardware and simulation.
 */
@ExtendWith(MockitoExtension.class)
class DriveTestAutoTest {

  @Mock private DriveSubsystem drive;
  @Mock private PoseEstimatorSubsystem pose;
  @Mock private StateSubsystem state;
  @Mock private PIDController rotPid;

  @BeforeAll
  static void initHAL() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void setup() {
    // The auto resets the drivetrain's rotation PID at completion
    lenient().when(drive.getRotPidController()).thenReturn(rotPid);
  }

  private void runFinalAction() throws Exception {
    DriveTestAuto auto = new DriveTestAuto(drive, pose, state);
    Field commandsField = auto.getClass().getSuperclass().getDeclaredField("m_commands");
    commandsField.setAccessible(true);
    @SuppressWarnings("unchecked")
    List<Command> commands = (List<Command>) commandsField.get(auto);
    Command finalCmd = commands.get(commands.size() - 1);
    finalCmd.initialize();
  }

  @Test
  void handlesShortSensorArrays() throws Exception {
    when(pose.getChassisSpeeds()).thenReturn(new ChassisSpeeds());
    when(pose.getCurrentPose()).thenReturn(new Pose2d());
    when(drive.getModuleCurrents()).thenReturn(new double[0]);
    when(drive.getDriveAppliedOutputs()).thenReturn(new double[0]);
    DriveTestAuto auto = new DriveTestAuto(drive, pose, state);
    Field commandsField = auto.getClass().getSuperclass().getDeclaredField("m_commands");
    commandsField.setAccessible(true);
    @SuppressWarnings("unchecked")
    List<Command> commands = (List<Command>) commandsField.get(auto);
    Command runCmd = commands.get(1);
    runCmd.initialize();
    SimHooks.stepTiming(0.25);
    assertDoesNotThrow(() -> runCmd.execute());
  }

  @Test
  void clearsLocksAfterCompletionOnRealRobot() throws Exception {
    SimHooks.setHALRuntimeType(HALUtil.RUNTIME_ROBORIO);
    runFinalAction();
    verify(drive).drive(0.0, 0.0, 0.0, false);
    verify(state).setRotationLock(false);
    verify(state).setDriveState(DriveState.Teleop);
    verify(rotPid).setSetpoint(0.0);
    verify(rotPid).reset();
  }

  @Test
  void clearsLocksAfterCompletionInSimulation() throws Exception {
    SimHooks.setHALRuntimeType(HALUtil.RUNTIME_SIMULATION);
    runFinalAction();
    verify(drive).drive(0.0, 0.0, 0.0, false);
    verify(state).setRotationLock(false);
    verify(state).setDriveState(DriveState.Teleop);
    verify(rotPid).setSetpoint(0.0);
    verify(rotPid).reset();
  }
}
