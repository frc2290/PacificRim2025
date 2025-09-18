// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    private final DriveStateMachine m_drive_state;
    private final ManipulatorStateMachine m_ManipulatorStateMachine;
    private final StateMachineCoardinator m_coardinator;

    // The driver's controller
    XboxController m_driverController;

    // Auto Chooser for Dashboard
    SendableChooser<Command> auto_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(LEDUtility _led, 
        DriveSubsystem _drive, 
        PoseEstimatorSubsystem _pose, 
        ElevatorSubsystem _elev,
        ManipulatorSubsystem _manip, 
        DifferentialSubsystem _diff, 
        ClimbSubsystem _climb, 
        DriveStateMachine _drive_state, 
        ManipulatorStateMachine _ManipulatorStateMachine, 
        StateMachineCoardinator _coardinator, 
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
        m_coardinator = _coardinator;

        // Configure the button bindings
        configureButtonBindings();

        m_ledUtility.addStrip("Left", 0, 61);
        m_ledUtility.addStrip("TopLeft", 62, 71);
        m_ledUtility.addStrip("Right", 72, 133);
        m_ledUtility.getStrip("Right").setHelperBool(true);
        m_ledUtility.addStrip("TopRight", 134, 143);
        m_ledUtility.setDefault();

        // Build an auto chooser. This will use Commands.none() as the default option.
        //auto_chooser.addOption("Drivetrain SysID", new DrivetrainSysId(m_robotDrive));
        //auto_chooser.addOption("Test", new Test(m_poseEstimator, m_state));
        //auto_chooser.addOption("Driving", new Auto(m_robotDrive));
        //auto_chooser.addOption("Right1Coral", new Right1Coral(m_DiffArm, m_poseEstimator, m_state, m_manipulator));
        //auto_chooser.addOption("RightCoral2", new Right2Coral(m_DiffArm, m_poseEstimator, m_state, m_manipulator));
        //auto_chooser.addOption("RightCoral3", new Right3Coral(m_DiffArm, m_poseEstimator, m_state, m_manipulator));
        //auto_chooser.addOption("Left3Coral", new Left3Coral(m_DiffArm, m_poseEstimator, m_state, m_manipulator));
        //auto_chooser.addOption("Middle1Coral", new Middle1Coral(m_DiffArm, m_poseEstimator, m_state, m_manipulator));
        SmartDashboard.putData(auto_chooser);


        //m_robotDrive.setDefaultCommand(new AutomatedDrive(m_state, m_robotDrive, m_DiffArm, m_poseEstimator, m_driverController));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {

        // Button definitions
        JoystickButton a_button = new JoystickButton(m_driverController, Button.kA.value);
        JoystickButton b_button = new JoystickButton(m_driverController, Button.kB.value);
        JoystickButton y_button = new JoystickButton(m_driverController, Button.kY.value);
        JoystickButton x_button = new JoystickButton(m_driverController, Button.kX.value);
        JoystickButton left_bumper = new JoystickButton(m_driverController, Button.kLeftBumper.value);
        JoystickButton right_bumper = new JoystickButton(m_driverController, Button.kRightBumper.value);
        JoystickButton back_button = new JoystickButton(m_driverController, Button.kBack.value);
        JoystickButton start_button = new JoystickButton(m_driverController, Button.kStart.value);
        JoystickButton left_stick = new JoystickButton(m_driverController, Button.kLeftStick.value);
        JoystickButton right_stick= new JoystickButton(m_driverController, Button.kRightStick.value);
        Trigger right_trigger = new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.5);
        Trigger left_trigger = new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5);
        POVButton dpad_up = new POVButton(m_driverController, 0);
        POVButton dpad_down = new POVButton(m_driverController, 180);
        POVButton dpad_left = new POVButton(m_driverController, 270);
        POVButton dpad_right = new POVButton(m_driverController, 90);

        Trigger not_left_stick = left_stick.negate(); // Trigger to check if left stick is not pressed in
        Trigger not_right_stick = right_stick.negate(); // Trigger to check if right stick is not pressed in
        Trigger coral_profilTrigger = new Trigger(() -> m_coardinator.getCurrentControllerProfile() == StateMachineCoardinator.ControllerProfile.DEFAULT_CORAL);
        Trigger algae_profilTrigger = new Trigger(() -> m_coardinator.getCurrentControllerProfile() == StateMachineCoardinator.ControllerProfile.ALGAE);    
        Trigger manual_profileTrigger = new Trigger(() -> m_coardinator.getCurrentControllerProfile() == StateMachineCoardinator.ControllerProfile.MANUAL);







        // Controller Buttons
        (a_button).and(coral_profilTrigger).onTrue(new InstantCommand(() -> m_coardinator.setRobotGoal(RobotState.INTAKE_CORAL))); // Set to L1
        (b_button).and(coral_profilTrigger).onTrue(new InstantCommand(() -> m_coardinator.setRobotGoal(RobotState.L2))); // Set to L2
        (y_button).and(coral_profilTrigger).onTrue(new InstantCommand(() -> m_coardinator.setRobotGoal(RobotState.L3))); // Set to L3
        (x_button).and(coral_profilTrigger).onTrue(new InstantCommand(() -> m_coardinator.setRobotGoal(RobotState.L4))); // Set to L4
        //back_button.onTrue(m_state.cancelCommand()); // Cancel current state  RECHECK
        //start_button.onTrue(m_state.toggleRotationLock()); // Toggle rotation lock for driver controls DO WE NEED THIS

        // Controller Bumpers
        (left_bumper).and(not_left_stick).onTrue(new InstantCommand(() -> m_coardinator.setRightScore(false))); // Set score to left branch
        (right_bumper).and(not_left_stick).onTrue(new InstantCommand(() -> m_coardinator.setRightScore(true))); // Set score to right branch
        
        // Controller D-Pad
        //dpad_up.and(not_right_stick).onTrue(m_state.setGoalElevManiCommand(ElevatorManipulatorState.AlgaeL3)); // Algae L3
        //dpad_down.and(not_right_stick).onTrue(m_state.setGoalElevManiCommand(ElevatorManipulatorState.AlgaeL2)); // Algae L2
        //dpad_right.and(not_right_stick).onTrue(m_manipulator.runIntake(-0.9)).onFalse(m_manipulator.runIntake(0)); //run intake
        
        // Controller Triggers
        left_trigger.onTrue(new InstantCommand(() -> m_coardinator.setReefAlign(true))).onFalse(new InstantCommand(() -> m_coardinator.setReefAlign(false))); //While held will try to align with the reef
        right_trigger.onTrue(new InstantCommand(() -> m_coardinator.requestToScore(true))).onFalse(new InstantCommand(() -> m_coardinator.requestToScore(false)));




        //IDK what is this, if that is logic, it should be inside state machine
        //hasCoral.or(hasAlgae).and(notAuto).onFalse(m_state.setGoalDriveCommand(DriveState.CoralStation)).onTrue(m_state.setGoalDriveCommand(DriveState.Teleop));

        // Manual controls
        dpad_up.and(not_right_stick).toggleOnTrue(new ParallelCommandGroup(new InstantCommand(() -> m_coardinator.setControllerProfile(ControllerProfile.DEFAULT_CORAL)), new InstantCommand(() -> m_coardinator.setRobotGoal(RobotState.SAFE_CORAL_TRAVEL)))); //coral mode
        dpad_down.and(not_right_stick).toggleOnTrue(new InstantCommand(() -> m_coardinator.setControllerProfile(ControllerProfile.ALGAE))); //algae mode
        dpad_left.and(not_right_stick).toggleOnTrue(new ParallelCommandGroup(new InstantCommand(() -> m_coardinator.setControllerProfile(ControllerProfile.MANUAL)), new InstantCommand(() -> m_coardinator.setRobotGoal(RobotState.MANUAL)))); // manula mode
        //dpad_right.and(not_right_stick).toggleOnTrue(new InstantCommand(() -> m_coardinator.setControllerProfile(ControllerProfile.MANUAL)));

        manual_profileTrigger.and(y_button).onTrue(m_elevator.incrementElevatorSetpoint(0.025)); // Manual move elevator up
        manual_profileTrigger.and(a_button).onTrue(m_elevator.incrementElevatorSetpoint(-0.025)); // Manual move elevator down
        manual_profileTrigger.and(x_button).onTrue(m_DiffArm.incrementExtensionSetpoint(5)); // Manual move diff arm out
        manual_profileTrigger.and(b_button).onTrue(m_DiffArm.incrementExtensionSetpoint(-5)); // Manual move diff arm in
        manual_profileTrigger.and(left_bumper).onTrue(m_DiffArm.incrementRotationSetpoint(5)); // Manual rotate diff arm out
        manual_profileTrigger.and(right_bumper).onTrue(m_DiffArm.incrementRotationSetpoint(-5)); // Manual rotate diff arm in
        
        //Other
        right_stick.and(dpad_left).onTrue(new InstantCommand(() -> m_climber.setServoOpen())); // Manual servo open
        right_stick.and(dpad_right).onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading())); // Manual heading reset
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