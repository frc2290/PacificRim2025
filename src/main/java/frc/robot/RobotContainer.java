// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos.ClimberTestAuto;
import frc.robot.commands.Autos.DifferentialTestAuto;
import frc.robot.commands.Autos.DriveTestAuto;
import frc.robot.commands.Autos.DrivetrainSysId;
import frc.robot.commands.Autos.ElevatorTestAuto;
import frc.robot.commands.Autos.ManipulatorTestAuto;
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

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final LEDUtility m_ledUtility;
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive;
  private final PoseEstimatorSubsystem m_poseEstimator;
  private final ElevatorSubsystem m_elevator;
  private final ManipulatorSubsystem m_manipulator;
  private final DifferentialSubsystem m_DiffArm;
  private final ClimbSubsystem m_climber;
  private final DriveStateMachine m_driveStateMachine;
  private final ManipulatorStateMachine m_manipulatorStateMachine;
  private final StateMachineCoardinator m_coordinator;

  // The driver's controller
  private final XboxController m_driverController;

  // Auto Chooser for Dashboard
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
      DriveStateMachine driveStateMachine,
      ManipulatorStateMachine manipulatorStateMachine,
      StateMachineCoardinator coordinator,
      XboxController driverController) {
    m_driverController = driverController;
    m_ledUtility = _led;
    m_robotDrive = _drive;
    m_poseEstimator = _pose;
    m_elevator = _elev;
    m_manipulator = _manip;
    m_DiffArm = _diff;
    m_climber = _climb;
    m_driveStateMachine = driveStateMachine;
    m_manipulatorStateMachine = manipulatorStateMachine;
    m_coordinator = coordinator;
    // Configure the button bindings
    configureButtonBindings();

    m_ledUtility.addStrip("Left", 0, 61);
    m_ledUtility.addStrip("TopLeft", 62, 71);
    m_ledUtility.addStrip("Right", 72, 133);
    m_ledUtility.getStrip("Right").setHelperBool(true);
    m_ledUtility.addStrip("TopRight", 134, 143);
    m_ledUtility.setDefault();

    auto_chooser.setDefaultOption("None", Commands.none());
    // Add autonomous options
    auto_chooser.addOption("Drivetrain SysID", new DrivetrainSysId(m_robotDrive));
    auto_chooser.addOption("Drive Test", new DriveTestAuto(m_robotDrive, m_poseEstimator));
    auto_chooser.addOption("Differential Test", new DifferentialTestAuto(m_DiffArm));
    auto_chooser.addOption("Elevator Test", new ElevatorTestAuto(m_elevator));
    auto_chooser.addOption("Manipulator Test", new ManipulatorTestAuto(m_manipulator));
    auto_chooser.addOption("Climber Test", new ClimberTestAuto(m_climber));
    SmartDashboard.putData(auto_chooser);
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
    Trigger coralProfile =
        new Trigger(
            () ->
                m_coordinator.getCurrentControllerProfile()
                    == StateMachineCoardinator.ControllerProfile.DEFAULT_CORAL);

    aButton
        .and(coralProfile)
        .onTrue(new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.SAFE_CORAL_TRAVEL)));
    bButton
        .and(coralProfile)
        .onTrue(new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.INTAKE_CORAL)));
    yButton
        .and(notLeftStick)
        .onTrue(new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.L4)));
    xButton
        .and(coralProfile)
        .onTrue(new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.PROCESSOR)));
    backButton.onTrue(new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.RESET)));
    startButton.onTrue(new InstantCommand(() -> m_coordinator.setRobotGoal(RobotState.MANUAL)));

    leftBumper
        .and(notLeftStick)
        .onTrue(new InstantCommand(() -> m_coordinator.setRightScore(false)));
    rightBumper
        .and(notLeftStick)
        .onTrue(new InstantCommand(() -> m_coordinator.setRightScore(true)));

    dpadRight
        .and(notRightStick)
        .onTrue(m_manipulator.runIntake(-0.9))
        .onFalse(m_manipulator.runIntake(0));

    leftTrigger
        .onTrue(new InstantCommand(() -> m_coordinator.setReefAlign(true)))
        .onFalse(new InstantCommand(() -> m_coordinator.setReefAlign(false)));
    rightTrigger
        .onTrue(new InstantCommand(() -> m_coordinator.score(true)))
        .onFalse(new InstantCommand(() -> m_coordinator.score(false)));

    rightStick.and(dpadLeft).onTrue(new InstantCommand(() -> m_climber.setServoOpen()));
    rightStick.and(dpadRight).onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto_chooser.getSelected();
  }

  /**
   * Runs each subsystem's simulation update. This is called from {@link Robot#simulationPeriodic()}
   * to advance the physics model when running in simulation.
   */
  public void simulationPeriodic() {
    m_robotDrive.simulationPeriodic();
    m_elevator.simulationPeriodic();
    m_manipulator.simulationPeriodic();
    m_DiffArm.simulationPeriodic();
    m_climber.simulationPeriodic();
  }
}
