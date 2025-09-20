// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
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
import frc.robot.commands.Autos.ClimberTestAuto;
import frc.robot.commands.Autos.DifferentialTestAuto;
import frc.robot.commands.Autos.DriveTestAuto;
import frc.robot.commands.Autos.DrivetrainSysId;
import frc.robot.commands.Autos.ElevatorTestAuto;
import frc.robot.commands.Autos.Left3Coral;
import frc.robot.commands.Autos.ManipulatorTestAuto;
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
import frc.robot.subsystems.StateMachineCoardinator;
import frc.robot.subsystems.StateMachineCoardinator.ControllerProfile;
import frc.robot.subsystems.StateMachineCoardinator.RobotState;
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
  private final DifferentialSubsystem m_differential;
  private final ClimbSubsystem m_climber;
  private final DriveStateMachine m_driveStateMachine;
  private final ManipulatorStateMachine m_manipulatorStateMachine;
  private final StateMachineCoardinator m_coordinator;

  // The driver's controller.
  private final XboxController m_driverController;

  /** Shuffleboard chooser that lets the drive team pick an auto routine each match. */
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(
      LEDUtility ledUtility,
      DriveSubsystem drive,
      PoseEstimatorSubsystem pose,
      ElevatorSubsystem elevator,
      ManipulatorSubsystem manipulator,
      DifferentialSubsystem differential,
      ClimbSubsystem climber,
      DriveStateMachine driveStateMachine,
      ManipulatorStateMachine manipulatorStateMachine,
      StateMachineCoardinator coordinator,
      XboxController driverController) {

    m_driverController = driverController;
    m_ledUtility = ledUtility;
    m_robotDrive = drive;
    m_poseEstimator = pose;
    m_elevator = elevator;
    m_manipulator = manipulator;
    m_differential = differential;
    m_climber = climber;
    m_driveStateMachine = driveStateMachine;
    m_manipulatorStateMachine = manipulatorStateMachine;
    m_coordinator = coordinator;

    configureButtonBindings();

    // Register each LED strip with the helper so state machines can update them by name.
    m_ledUtility.addStrip("Left", 0, 61);
    m_ledUtility.addStrip("TopLeft", 62, 71);
    m_ledUtility.addStrip("Right", 72, 133);
    m_ledUtility.getStrip("Right").setHelperBool(true);
    m_ledUtility.addStrip("TopRight", 134, 143);
    m_ledUtility.setDefault();

    configureAutoChooser();
  }

  private void configureAutoChooser() {
    if (RobotBase.isSimulation()) {
      m_autoChooser.setDefaultOption(
          "Drive Test (Full Speed)",
          new DriveTestAuto(
              m_robotDrive, m_poseEstimator, DriveTestAuto.FULL_SPEED_TRANSLATION_SCALAR));
      m_autoChooser.addOption("None", Commands.none());
    } else {
      m_autoChooser.setDefaultOption("None", Commands.none());
      m_autoChooser.addOption(
          "Drive Test (Full Speed)",
          new DriveTestAuto(
              m_robotDrive, m_poseEstimator, DriveTestAuto.FULL_SPEED_TRANSLATION_SCALAR));
    }

    // Competition autos from the state machine branch.
    m_autoChooser.addOption(
        "Right1Coral",
        new Right1Coral(
            m_poseEstimator,
            m_driveStateMachine,
            m_coordinator,
            m_manipulatorStateMachine,
            m_manipulator));
    m_autoChooser.addOption(
        "Right2Coral",
        new Right2Coral(
            m_poseEstimator,
            m_driveStateMachine,
            m_coordinator,
            m_manipulatorStateMachine,
            m_manipulator));
    m_autoChooser.addOption(
        "Right3Coral",
        new Right3Coral(
            m_poseEstimator,
            m_driveStateMachine,
            m_coordinator,
            m_manipulatorStateMachine,
            m_manipulator));
    m_autoChooser.addOption(
        "Left3Coral",
        new Left3Coral(
            m_poseEstimator,
            m_driveStateMachine,
            m_coordinator,
            m_manipulatorStateMachine,
            m_manipulator));
    m_autoChooser.addOption(
        "Middle1Coral",
        new Middle1Coral(
            m_poseEstimator,
            m_driveStateMachine,
            m_coordinator,
            m_manipulatorStateMachine,
            m_manipulator));

    // Diagnostic and characterization routines useful during development.
    m_autoChooser.addOption("Drivetrain SysID", new DrivetrainSysId(m_robotDrive));
    m_autoChooser.addOption("Drive Test", new DriveTestAuto(m_robotDrive, m_poseEstimator));
    m_autoChooser.addOption("Differential Test", new DifferentialTestAuto(m_differential));
    m_autoChooser.addOption("Elevator Test", new ElevatorTestAuto(m_elevator));
    m_autoChooser.addOption("Manipulator Test", new ManipulatorTestAuto(m_manipulator));
    m_autoChooser.addOption("Climber Test", new ClimberTestAuto(m_climber));

    SmartDashboard.putData(m_autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton aButton = new JoystickButton(m_driverController, Button.kA.value);
    JoystickButton bButton = new JoystickButton(m_driverController, Button.kB.value);
    JoystickButton yButton = new JoystickButton(m_driverController, Button.kY.value);
    JoystickButton xButton = new JoystickButton(m_driverController, Button.kX.value);
    JoystickButton leftBumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
    JoystickButton rightBumper = new JoystickButton(m_driverController, Button.kRightBumper.value);
    JoystickButton backButton = new JoystickButton(m_driverController, Button.kBack.value);
    JoystickButton startButton = new JoystickButton(m_driverController, Button.kStart.value);
    JoystickButton leftStick = new JoystickButton(m_driverController, Button.kLeftStick.value);
    JoystickButton rightStick = new JoystickButton(m_driverController, Button.kRightStick.value);
    Trigger rightTrigger = new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.5);
    Trigger leftTrigger = new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5);
    POVButton dpadUp = new POVButton(m_driverController, 0);
    POVButton dpadDown = new POVButton(m_driverController, 180);
    POVButton dpadLeft = new POVButton(m_driverController, 270);
    POVButton dpadRight = new POVButton(m_driverController, 90);

    Trigger notLeftStick = leftStick.negate();
    Trigger notRightStick = rightStick.negate();
    // Profile triggers make it easy to reuse button bindings while the controller changes modes.
    Trigger coralProfile =
        new Trigger(
            () -> m_coordinator.getCurrentControllerProfile() == ControllerProfile.DEFAULT_CORAL);
    Trigger algaeProfile =
        new Trigger(() -> m_coordinator.getCurrentControllerProfile() == ControllerProfile.ALGAE);
    Trigger manualProfile =
        new Trigger(() -> m_coordinator.getCurrentControllerProfile() == ControllerProfile.MANUAL);

    // Controller buttons.
    aButton
        .and(coralProfile)
        .onTrue(new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.INTAKE_CORAL)));
    bButton
        .and(coralProfile)
        .onTrue(new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.L2)));
    yButton
        .and(coralProfile)
        .onTrue(new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.L3)));
    xButton
        .and(coralProfile)
        .onTrue(new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.L4)));
    backButton.onTrue(new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.RESET)));
    startButton.onTrue(new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.MANUAL)));

    // Controller bumpers select the scoring side.
    leftBumper
        .and(notLeftStick)
        .onTrue(new InstantCommand(() -> m_coordinator.setRightScore(false)));
    rightBumper
        .and(notLeftStick)
        .onTrue(new InstantCommand(() -> m_coordinator.setRightScore(true)));

    // Controller triggers.
    leftTrigger
        .onTrue(new InstantCommand(() -> m_coordinator.setReefAlign(true)))
        .onFalse(new InstantCommand(() -> m_coordinator.setReefAlign(false)));
    rightTrigger
        .onTrue(new InstantCommand(() -> m_coordinator.requestToScore(true)))
        .onFalse(new InstantCommand(() -> m_coordinator.requestToScore(false)));

    // Manual controls.
    dpadUp
        .and(notRightStick)
        .toggleOnTrue(
            new ParallelCommandGroup(
                new InstantCommand(
                    () -> m_coordinator.setControllerProfile(ControllerProfile.DEFAULT_CORAL)),
                new InstantCommand(
                    () -> m_coordinator.setRobotGoal(RobotState.SAFE_CORAL_TRAVEL))));
    dpadDown
        .and(notRightStick)
        .toggleOnTrue(
            new InstantCommand(() -> m_coordinator.setControllerProfile(ControllerProfile.ALGAE)));
    dpadLeft
        .and(notRightStick)
        .toggleOnTrue(
            new ParallelCommandGroup(
                new InstantCommand(
                    () -> m_coordinator.setControllerProfile(ControllerProfile.MANUAL)),
                new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.MANUAL))));

    manualProfile.and(yButton).onTrue(m_elevator.incrementElevatorSetpoint(0.025));
    manualProfile.and(aButton).onTrue(m_elevator.incrementElevatorSetpoint(-0.025));
    manualProfile.and(xButton).onTrue(m_differential.incrementExtensionSetpoint(5));
    manualProfile.and(bButton).onTrue(m_differential.incrementExtensionSetpoint(-5));
    manualProfile.and(leftBumper).onTrue(m_differential.incrementRotationSetpoint(5));
    manualProfile.and(rightBumper).onTrue(m_differential.incrementRotationSetpoint(-5));

    // Other controls.
    rightStick.and(dpadLeft).onTrue(new InstantCommand(() -> m_climber.setServoOpen()));
    rightStick.and(dpadRight).onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  /**
   * Runs each subsystem's simulation update. This is called from {@link Robot#simulationPeriodic()}
   * to advance the physics model when running in simulation.
   */
  public void simulationPeriodic() {
    m_robotDrive.simulationPeriodic();
    m_elevator.simulationPeriodic();
    m_manipulator.simulationPeriodic();
    m_differential.simulationPeriodic();
    m_climber.simulationPeriodic();
  }
}
