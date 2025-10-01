// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.DriveCommands.DriveCommandFactory;
import frc.robot.commands.GraphCommand;
import frc.robot.commands.GraphCommand.GraphCommandNode;
import frc.utils.PoseEstimatorSubsystem;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import sun.misc.Unsafe;

class DriveStateMachineTest {

  private static final Unsafe unsafe;

  static {
    try {
      Field f = Unsafe.class.getDeclaredField("theUnsafe");
      f.setAccessible(true);
      unsafe = (Unsafe) f.get(null);
    } catch (ReflectiveOperationException e) {
      throw new ExceptionInInitializerError(e);
    }
  }

  private final CommandScheduler scheduler = CommandScheduler.getInstance();

  @BeforeEach
  void resetScheduler() {
    scheduler.cancelAll();
  }

  @AfterEach
  void cleanupScheduler() {
    scheduler.cancelAll();
  }

  @Test
  void climbRelativeSchedulesHeadingLock() throws Exception {
    assertTrue(edu.wpi.first.hal.HAL.initialize(500, 0));
    scheduler.enable();
    StubDriveSubsystem drive = allocate(StubDriveSubsystem.class);
    StubPoseEstimatorSubsystem pose = allocate(StubPoseEstimatorSubsystem.class);
    pose.setDegrees(30.0);
    StubXboxController controller = allocate(StubXboxController.class);

    DriveCommandFactory factory = allocate(DriveCommandFactory.class);
    setObjectField(factory, "drive", drive);
    setObjectField(factory, "poseEstimator", pose);
    setObjectField(factory, "driverController", controller);
    setObjectField(factory, "rotPid", drive.rotPid);
    setObjectField(factory, "xPid", drive.xPid);
    setObjectField(factory, "yPid", drive.yPid);

    DriveStateMachine machine = allocate(DriveStateMachine.class);
    GraphCommand graphCommand = new GraphCommand();
    setObjectField(machine, "m_graphCommand", graphCommand);
    setObjectField(machine, "dashboard", new frc.utils.FlytDashboardV2("DriveStateMachineTest"));
    setObjectField(machine, "drive", drive);
    setObjectField(machine, "pose", pose);
    setObjectField(machine, "driverController", controller);
    setObjectField(machine, "driveCommandFactory", factory);
    setDoubleField(machine, "bargeHeadingDegrees", 0.0);
    setDoubleField(machine, "climbHeadingDegrees", 0.0);
    setBooleanField(machine, "rightScore", false);

    Method initialize = DriveStateMachine.class.getDeclaredMethod("initializeGraphCommand");
    initialize.setAccessible(true);
    initialize.invoke(machine);

    scheduler.registerSubsystem(machine);
    scheduler.registerSubsystem(drive);
    addRequirements(graphCommand, machine);

    GraphCommandNode cancelledNode = (GraphCommandNode) getObjectField(machine, "cancelledNode");
    graphCommand.setCurrentNode(cancelledNode);

    graphCommand.initialize();

    machine.setDriveCommand(DriveStateMachine.DriveState.CLIMB_RELATIVE);

    GraphCommandNode climbNode = (GraphCommandNode) getObjectField(machine, "climbRelativeNode");

    graphCommand.execute();

    Command headingLockCommand = (Command) getObjectField(climbNode, "m_targetCommand");

    assertTrue(
        graphCommand.getCurrentNode() == climbNode,
        "GraphCommand should reach the climb-relative node");

    assertTrue(headingLockCommand != null, "Climb node should provide a heading lock command");
  }

  private static <T> T allocate(Class<T> type) throws InstantiationException {
    return type.cast(unsafe.allocateInstance(type));
  }

  private static void setObjectField(Object target, String name, Object value) throws Exception {
    Field field = target.getClass().getDeclaredField(name);
    field.setAccessible(true);
    unsafe.putObject(target, unsafe.objectFieldOffset(field), value);
  }

  private static Object getObjectField(Object target, String name) throws Exception {
    Field field = target.getClass().getDeclaredField(name);
    field.setAccessible(true);
    return unsafe.getObject(target, unsafe.objectFieldOffset(field));
  }

  private static void setDoubleField(Object target, String name, double value) throws Exception {
    Field field = target.getClass().getDeclaredField(name);
    field.setAccessible(true);
    unsafe.putDouble(target, unsafe.objectFieldOffset(field), value);
  }

  private static void setBooleanField(Object target, String name, boolean value) throws Exception {
    Field field = target.getClass().getDeclaredField(name);
    field.setAccessible(true);
    unsafe.putBoolean(target, unsafe.objectFieldOffset(field), value);
  }

  private static void addRequirements(Command command, Subsystem... subsystems) throws Exception {
    Method addRequirements = Command.class.getDeclaredMethod("addRequirements", Subsystem[].class);
    addRequirements.setAccessible(true);
    addRequirements.invoke(command, new Object[] {subsystems});
  }

  private static class StubDriveSubsystem extends DriveSubsystem {
    final PIDController rotPid = new PIDController(0.0, 0.0, 0.0);
    final PIDController xPid = new PIDController(0.0, 0.0, 0.0);
    final PIDController yPid = new PIDController(0.0, 0.0, 0.0);
    boolean headingLockScheduled = false;

    @Override
    public PIDController getRotPidController() {
      return rotPid;
    }

    @Override
    public PIDController getXPidController() {
      return xPid;
    }

    @Override
    public PIDController getYPidController() {
      return yPid;
    }

    @Override
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
      headingLockScheduled = true;
    }

    @Override
    public void periodic() {
      // Skip publishing drivetrain telemetry during tests.
    }
  }

  private static class StubPoseEstimatorSubsystem extends PoseEstimatorSubsystem {
    private StubPoseEstimatorSubsystem() {
      super((DriveSubsystem) null);
    }

    private Pose2d currentPose = new Pose2d();
    private Pose2d targetPose = new Pose2d();
    private Pose2d closestBranch = new Pose2d();
    private double degrees = 0.0;
    private boolean closestStationRight = false;

    @Override
    public double getDegrees() {
      return degrees;
    }

    @Override
    public Pose2d getCurrentPose() {
      return currentPose;
    }

    @Override
    public void setTargetPose(Pose2d newTarget) {
      targetPose = newTarget;
    }

    @Override
    public Pose2d getTargetPose() {
      return targetPose;
    }

    @Override
    public double turnToTarget(Translation2d target) {
      return 0.0;
    }

    @Override
    public boolean atTargetPose() {
      return true;
    }

    @Override
    public boolean isClosestStationRight() {
      return closestStationRight;
    }

    @Override
    public Pose2d getClosestBranch(boolean right) {
      return closestBranch;
    }

    void setDegrees(double degrees) {
      this.degrees = degrees;
    }

    void setClosestBranch(Pose2d branch) {
      this.closestBranch = branch;
    }

    void setClosestStationRight(boolean value) {
      this.closestStationRight = value;
    }
  }

  private static class StubXboxController extends XboxController {
    private StubXboxController() {
      super(0);
    }

    @Override
    public double getLeftY() {
      return 0.0;
    }

    @Override
    public double getLeftX() {
      return 0.0;
    }

    @Override
    public double getRightX() {
      return 0.0;
    }
  }
}
