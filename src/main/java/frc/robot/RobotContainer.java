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

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos.Left3Coral;
import frc.robot.commands.Autos.Middle1Coral;
import frc.robot.commands.Autos.Right1Coral;
import frc.robot.commands.Autos.Right2Coral;
import frc.robot.commands.Autos.Right3Coral;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.DriveStateMachine;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateMachineCoordinator;
import frc.robot.subsystems.StateMachineCoordinator.ControllerProfile;
import frc.robot.subsystems.StateMachineCoordinator.RobotState;
import frc.utils.LEDUtility;
import frc.utils.PoseEstimatorSubsystem;

/**
 * Central wiring hub for Excelsior. This container builds subsystems, registers default commands,
 * publishes autonomous routines, and binds the driver controller so the rest of the code can focus
 * on describing behaviors instead of input plumbing.
 */
public class RobotContainer {
  private final LEDUtility m_ledUtility;
  // The robot's subsystems.
  private final DriveSubsystem m_robotDrive;
  private final PoseEstimatorSubsystem m_poseEstimator;
  private final ElevatorSubsystem m_elevator;
  private final ManipulatorSubsystem m_manipulator;
  private final DifferentialSubsystem m_DiffArm;
  private final ClimbSubsystem m_climber;
  private final DriveStateMachine m_drive_state;
  private final ManipulatorStateMachine m_ManipulatorStateMachine;
  private final StateMachineCoordinator m_coordinator;

  // The driver's controller
  /** Driver joystick used for all manual control. */
  XboxController m_driverController;

  /** Shuffleboard chooser that lets the drive team pick an auto routine each match. */
  SendableChooser<Command> auto_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(
      LEDUtility _led,
      DriveSubsystem _drive,
      PoseEstimatorSubsystem _pose,
      ElevatorSubsystem _elev,
      ManipulatorSubsystem _manip,
      DifferentialSubsystem _diff,
      ClimbSubsystem _climb,
      DriveStateMachine _drive_state,
      ManipulatorStateMachine _ManipulatorStateMachine,
      StateMachineCoordinator _coordinator,
      XboxController _driverController) {

    m_driverController = _driverController;
    m_ledUtility = _led;
    m_robotDrive = _drive;
    m_poseEstimator = _pose;
    m_elevator = _elev;
    m_manipulator = _manip;
    m_DiffArm = _diff;
    m_climber = _climb;
    m_drive_state = _drive_state;
    m_ManipulatorStateMachine = _ManipulatorStateMachine;
    m_coordinator = _coordinator;

    // Configure the button bindings.
    configureButtonBindings();

    // Register each LED strip with the helper so state machines can update them by name.
    m_ledUtility.addStrip("Left", 0, 61);
    m_ledUtility.addStrip("TopLeft", 62, 71);
    m_ledUtility.addStrip("Right", 72, 133);
    m_ledUtility.getStrip("Right").setHelperBool(true);
    m_ledUtility.addStrip("TopRight", 134, 143);
    m_ledUtility.setDefault();

    // Build an auto chooser. This will use Commands.none() as the default option.
    auto_chooser.setDefaultOption("None", Commands.none());
    auto_chooser.addOption(
        "Right1Coral",
        new Right1Coral(
            m_poseEstimator,
            m_drive_state,
            m_coordinator,
            m_ManipulatorStateMachine,
            m_manipulator));
    auto_chooser.addOption(
        "Right2Coral",
        new Right2Coral(
            m_poseEstimator,
            m_drive_state,
            m_coordinator,
            m_ManipulatorStateMachine,
            m_manipulator));
    auto_chooser.addOption(
        "Right3Coral",
        new Right3Coral(
            m_poseEstimator,
            m_drive_state,
            m_coordinator,
            m_ManipulatorStateMachine,
            m_manipulator));
    auto_chooser.addOption(
        "Left3Coral",
        new Left3Coral(
            m_poseEstimator,
            m_drive_state,
            m_coordinator,
            m_ManipulatorStateMachine,
            m_manipulator));
    auto_chooser.addOption(
        "Middle1Coral",
        new Middle1Coral(
            m_poseEstimator,
            m_drive_state,
            m_coordinator,
            m_ManipulatorStateMachine,
            m_manipulator));
    // Expose the options to the dashboard so the drive team can select before each match.
    SmartDashboard.putData(auto_chooser);

    // m_robotDrive.setDefaultCommand(new AutomatedDrive(m_state, m_robotDrive, m_DiffArm,
    // m_poseEstimator, m_driverController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Button definitions.
    // Map the raw controller buttons to descriptive names for readability.
    JoystickButton a_button = new JoystickButton(m_driverController, Button.kA.value);
    JoystickButton b_button = new JoystickButton(m_driverController, Button.kB.value);
    JoystickButton y_button = new JoystickButton(m_driverController, Button.kY.value);
    JoystickButton x_button = new JoystickButton(m_driverController, Button.kX.value);
    JoystickButton left_bumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
    JoystickButton right_bumper = new JoystickButton(m_driverController, Button.kRightBumper.value);
    JoystickButton back_button = new JoystickButton(m_driverController, Button.kBack.value);
    JoystickButton start_button = new JoystickButton(m_driverController, Button.kStart.value);
    JoystickButton left_stick = new JoystickButton(m_driverController, Button.kLeftStick.value);
    JoystickButton right_stick = new JoystickButton(m_driverController, Button.kRightStick.value);
    Trigger right_trigger = new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.5);
    Trigger left_trigger = new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5);
    POVButton dpad_up = new POVButton(m_driverController, 0);
    POVButton dpad_down = new POVButton(m_driverController, 180);
    POVButton dpad_left = new POVButton(m_driverController, 270);
    POVButton dpad_right = new POVButton(m_driverController, 90);

    Trigger not_left_stick = left_stick.negate(); // Trigger to check if left stick is not pressed in.
    Trigger not_right_stick = right_stick.negate(); // Trigger to check if right stick is not pressed in.
    // Profile triggers make it easy to reuse button bindings while the controller changes modes.
    Trigger coral_profileTrigger = new Trigger(() -> m_coordinator.getCurrentControllerProfile() == StateMachineCoordinator.ControllerProfile.DEFAULT_CORAL);
    Trigger algae_profileTrigger = new Trigger(() -> m_coordinator.getCurrentControllerProfile() == StateMachineCoordinator.ControllerProfile.ALGAE);
    Trigger manual_profileTrigger = new Trigger(() -> m_coordinator.getCurrentControllerProfile() == StateMachineCoordinator.ControllerProfile.MANUAL);

    // Controller buttons.
    //(a_button).and(coral_profileTrigger).onTrue(new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.L1))); // Request the intake coral routine.
    (b_button).and(coral_profileTrigger).onTrue(new InstantCommand(() ->m_coordinator.setRobotGoal(RobotState.L2))); // Request the L2 scoring routine.
    (y_button).and(coral_profileTrigger).onTrue(new InstantCommand(() ->m_coordinator.setRobotGoal(RobotState.L3))); // Request the L3 scoring routine.
    (x_button).and(coral_profileTrigger).onTrue(new InstantCommand(() ->m_coordinator.setRobotGoal(RobotState.L4))); // Request the L4 scoring routine.

    (a_button).and(coral_profileTrigger).onTrue(new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.PROCESSOR))); // Request Score Algage Processor
    (b_button).and(algae_profileTrigger).onTrue(new InstantCommand(() ->m_coordinator.setRobotGoal(RobotState.ALGAE_L2))); // Request Intake Algae L2
    (y_button).and(algae_profileTrigger).onTrue(new InstantCommand(() ->m_coordinator.setRobotGoal(RobotState.ALGAE_L3))); // Request Intake Algae L3
    (x_button).and(algae_profileTrigger).onTrue(new InstantCommand(() ->m_coordinator.setRobotGoal(RobotState.BARGE))); // Request Barge

    // Controller bumpers.
    (left_bumper).onTrue(new InstantCommand(() -> m_coordinator.setRightScore(false))); // Select the left reef branch.
    (right_bumper).onTrue(new InstantCommand(() -> m_coordinator.setRightScore(true))); // Select the right reef branch.

    // Controller triggers.
    left_trigger.onTrue(new InstantCommand(() -> m_coordinator.setReefAlign(true))).onFalse(new InstantCommand(() ->m_coordinator.setReefAlign(false))); // While held the robot tries to align with the reef.
    right_trigger.onTrue(new InstantCommand(() -> m_coordinator.requestToScore(true))).onFalse(new InstantCommand(() -> m_coordinator.requestToScore(false)));

    // Legacy logic for automatically switching modes lives in the state machine now.
    // hasCoral.or(hasAlgae).and(notAuto).onFalse(m_state.setGoalDriveCommand(DriveState.CoralStation)).onTrue(m_state.setGoalDriveCommand(DriveState.Teleop));

    // Manual controls.
    dpad_up.toggleOnTrue(new ParallelCommandGroup(
        new InstantCommand(() -> m_coordinator.setControllerProfile(ControllerProfile.DEFAULT_CORAL)),
        new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.SAFE_CORAL_TRANSPORT)))); // Coral profile with safe travel goal.

    dpad_down.toggleOnTrue(new ParallelCommandGroup(
        new InstantCommand(() -> m_coordinator.setControllerProfile(ControllerProfile.ALGAE)),
        new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.SAFE_ALGAE_TRANSPORT)))); // Algae profile with safe travel goal.

    dpad_left.toggleOnTrue(new ParallelCommandGroup(
                new InstantCommand(() -> m_coordinator.setControllerProfile(ControllerProfile.MANUAL)),
                new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.MANUAL)))); // Manual profile.

    dpad_right.and(start_button).toggleOnTrue(new ParallelCommandGroup( //against accidental presses
        new InstantCommand(() -> m_coordinator.setControllerProfile(ControllerProfile.Climb)), //this is for protection against scoring in climb profile
        new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.MANUAL)))); // Manual profile.


    manual_profileTrigger
        .and(y_button)
        .onTrue(m_elevator.incrementElevatorSetpoint(0.025)); // Manual move elevator up.
    manual_profileTrigger
        .and(a_button)
        .onTrue(m_elevator.incrementElevatorSetpoint(-0.025)); // Manual move elevator down.
    manual_profileTrigger
        .and(x_button)
        .onTrue(m_DiffArm.incrementExtensionSetpoint(5)); // Manual move diff arm out.
    manual_profileTrigger
        .and(b_button)
        .onTrue(m_DiffArm.incrementExtensionSetpoint(-5)); // Manual move diff arm in.
    manual_profileTrigger
        .and(left_bumper)
        .onTrue(m_DiffArm.incrementRotationSetpoint(5)); // Manual rotate diff arm out.
    manual_profileTrigger
        .and(right_bumper)
        .onTrue(m_DiffArm.incrementRotationSetpoint(-5)); // Manual rotate diff arm in.

    // Other controls.
    right_stick.and(dpad_left).onTrue(new InstantCommand(() -> m_climber.setServoOpen())); // Manual servo open.
    right_stick.and(dpad_right).onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading())); // Manual heading reset.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto_chooser.getSelected();
  }
}
