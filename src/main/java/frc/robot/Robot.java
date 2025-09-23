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
package frc.robot;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.DriveStateMachine;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateMachineCoardinator;
import frc.robot.subsystems.StateMachineCoardinator.RobotState;
import frc.utils.LEDUtility;
import frc.utils.PoseEstimatorSubsystem;
import org.littletonrobotics.urcl.URCL;

/**
 * Primary program entry point for Excelsior. {@link TimedRobot} periodically calls each lifecycle
 * hook so we can schedule commands, monitor subsystems, and react to driver inputs. The goal of
 * this class is to orchestrate subsystem construction and defer logic to the command-based
 * framework.
 */
public class Robot extends TimedRobot {
  /** Cached handle to the active autonomous routine so we can cancel or reschedule if needed. */
  private Command m_autonomousCommand;

  /** Container that wires up all subsystems, commands, and driver controls. */
  private RobotContainer m_robotContainer;

  /** Dedicated LED helper that allows the state machine to communicate robot status. */
  private final LEDUtility m_ledUtility = new LEDUtility(0);

  /** Cached reference to the primary driver controller so subsystems can read axes. */
  private XboxController m_driver = new XboxController(0);

  // The robot's subsystems.
  /** Owns all hardware for swerve driving and exposes the drive commands. */
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  /** Combines vision and gyro data to maintain the best guess at the robot's pose. */
  private final PoseEstimatorSubsystem m_poseEstimator = new PoseEstimatorSubsystem(m_robotDrive);

  /** Elevator carriage that moves the manipulator up and down the reef. */
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();

  /** Roller intake and sensors that interact directly with game pieces. */
  private final ManipulatorSubsystem m_manipulator = new ManipulatorSubsystem();

  /** Differential arm that provides extension and rotation for the manipulator. */
  private final DifferentialSubsystem m_DiffArm = new DifferentialSubsystem();

  /** Hooking subsystem used during endgame climbs. */
  private final ClimbSubsystem m_climber = new ClimbSubsystem();

  /** Coordinates all autonomous and teleop driving modes. */
  private final DriveStateMachine m_driveStateMachine =
      new DriveStateMachine(m_robotDrive, m_poseEstimator, m_driver);

  /** Tracks and sequences manipulator states to guarantee safe transitions. */
  private final ManipulatorStateMachine m_manipulatorStateMachine =
      new ManipulatorStateMachine(m_DiffArm, m_elevator, m_manipulator, m_climber);

  /** Central coordinator that keeps drive and manipulator state machines in sync. */
  private final StateMachineCoardinator m_coardinator =
      new StateMachineCoardinator(m_manipulatorStateMachine, m_driveStateMachine, m_ledUtility);

  public Robot() {
    // Ensure the CANivore bridge is connected before any CAN devices are created.
    CanBridge.runTCP();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate the container so bindings, chooser registration, and default commands run once.
    // All command bindings and default commands live in the container; create it once here.
    m_robotContainer =
        new RobotContainer(
            m_ledUtility,
            m_robotDrive,
            m_poseEstimator,
            m_elevator,
            m_manipulator,
            m_DiffArm,
            m_climber,
            m_driveStateMachine,
            m_manipulatorStateMachine,
            m_coardinator,
            m_driver);
    DataLogManager.start();

    // Start the logging framework so we can view graphs after a match or practice run.
    URCL.start();

    // If logging only to DataLog.
    URCL.start(DataLogManager.getLog());

    // Display the Command Scheduler Status
    SmartDashboard.putData(CommandScheduler.getInstance());

    // Display Subsystem Status
    SmartDashboard.putData(m_elevator);
    SmartDashboard.putData(m_DiffArm);
    SmartDashboard.putData(m_manipulator);
    SmartDashboard.putData(m_robotDrive);
    SmartDashboard.putData(m_manipulatorStateMachine);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Run the scheduler to poll buttons, execute commands, and invoke subsystem periodic methods.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // Notify subsystems that the robot is disabled so they can stop any motion and update LEDs.
    m_coardinator.robotDisabled(true);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Mark that the robot is in autonomous so the state machines can choose the correct defaults.
    m_coardinator.robotAuto(true);
    m_coardinator.robotDisabled(false);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // Schedule the autonomous command if one was selected.
    if (m_autonomousCommand != null) {
      // Defer command execution to the scheduler so it can manage interruptions properly.
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_coardinator.robotAuto(false);
    m_coardinator.robotDisabled(false);
    // Reset both state machines back to their safe starting configuration for teleop.
    m_coardinator.setRobotGoal(RobotState.START_POSITION);
    // Reset hook for post-autonomous tweaks if we ever need to retune teleop initialization.
    // m_state.setCurrentElevManiStateCommand(ElevatorManipulatorState.SafeCoralTravel);

    // Uncomment if we ever need to forcefully cancel the autonomous command during teleop init.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
