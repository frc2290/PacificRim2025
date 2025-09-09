// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
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
import frc.robot.commands.Auto;
import frc.robot.commands.Autos.DrivetrainSysId;
import frc.robot.commands.Autos.Test;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.DriveStateMachine;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorManipulatorStateMachine;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateMachineCoordinator;
import frc.utils.LEDUtility;
import frc.utils.PoseEstimatorSubsystem;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {
    // Subsystems
    private final LEDUtility ledUtility;
    private final DriveSubsystem drive;
    private final PoseEstimatorSubsystem poseEstimator;
    private final ElevatorSubsystem elevator;
    private final ManipulatorSubsystem manipulator;
    private final DifferentialSubsystem diffArm;
    private final ClimbSubsystem climber;
    
    // State Machines
    private final ElevatorManipulatorStateMachine elevManiStateMachine;
    private final DriveStateMachine driveStateMachine;
    private final StateMachineCoordinator stateMachineCoordinator;
    
    // Controller
    private final XboxController driverController;
    
    // Auto Chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(LEDUtility ledUtility, DriveSubsystem drive, 
                         PoseEstimatorSubsystem poseEstimator, ElevatorSubsystem elevator,
                         ManipulatorSubsystem manipulator, DifferentialSubsystem diffArm,
                         ClimbSubsystem climber, ElevatorManipulatorStateMachine elevManiStateMachine,
                         DriveStateMachine driveStateMachine, StateMachineCoordinator stateMachineCoordinator,
                         XboxController driverController) {
        
        // Initialize subsystems and state machines
        this.ledUtility = ledUtility;
        this.drive = drive;
        this.poseEstimator = poseEstimator;
        this.elevator = elevator;
        this.manipulator = manipulator;
        this.diffArm = diffArm;
        this.climber = climber;
        this.elevManiStateMachine = elevManiStateMachine;
        this.driveStateMachine = driveStateMachine;
        this.stateMachineCoordinator = stateMachineCoordinator;
        this.driverController = driverController;

        // Configure button bindings and setup
        configureButtonBindings();
        setupLEDs();
        setupAutoChooser();
        configureDefaultCommands();
    }

    /**
     * Configure LED strips
     */
    private void setupLEDs() {
        ledUtility.addStrip("Left", 0, 61);
        ledUtility.addStrip("TopLeft", 62, 71);
        ledUtility.addStrip("Right", 72, 133);
        ledUtility.getStrip("Right").setHelperBool(true);
        ledUtility.addStrip("TopRight", 134, 143);
        ledUtility.setDefault();
    }

    /**
     * Setup autonomous chooser
     */
    private void setupAutoChooser() {
        autoChooser.addOption("Drivetrain SysID", new DrivetrainSysId(drive));
        //autoChooser.addOption("Test", new Test(poseEstimator, elevManiStateMachine));
        autoChooser.addOption("Driving", new Auto(drive));
        
        // Commented out auto options for future use
        // autoChooser.addOption("Right1Coral", new Right1Coral(diffArm, poseEstimator, elevManiStateMachine, manipulator));
        // autoChooser.addOption("RightCoral2", new Right2Coral(diffArm, poseEstimator, elevManiStateMachine, manipulator));
        // autoChooser.addOption("RightCoral3", new Right3Coral(diffArm, poseEstimator, elevManiStateMachine, manipulator));
        // autoChooser.addOption("Left3Coral", new Left3Coral(diffArm, poseEstimator, elevManiStateMachine, manipulator));
        // autoChooser.addOption("Middle1Coral", new Middle1Coral(diffArm, poseEstimator, elevManiStateMachine, manipulator));
        
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Configure default commands for subsystems
     */
    private void configureDefaultCommands() {
        // Drive default command will be handled by DriveStateMachine
        // Elevator/Manipulator default command handled by GraphCommand
    }

    /**
     * Configure button bindings
     */
    private void configureButtonBindings() {
        // Button definitions
        JoystickButton aButton = new JoystickButton(driverController, Button.kA.value);
        JoystickButton bButton = new JoystickButton(driverController, Button.kB.value);
        JoystickButton yButton = new JoystickButton(driverController, Button.kY.value);
        JoystickButton xButton = new JoystickButton(driverController, Button.kX.value);
        JoystickButton leftBumper = new JoystickButton(driverController, Button.kLeftBumper.value);
        JoystickButton rightBumper = new JoystickButton(driverController, Button.kRightBumper.value);
        JoystickButton backButton = new JoystickButton(driverController, Button.kBack.value);
        JoystickButton startButton = new JoystickButton(driverController, Button.kStart.value);
        JoystickButton leftStick = new JoystickButton(driverController, Button.kLeftStick.value);
        JoystickButton rightStick = new JoystickButton(driverController, Button.kRightStick.value);
        
        Trigger rightTrigger = new Trigger(() -> driverController.getRightTriggerAxis() > 0.5);
        Trigger leftTrigger = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.5);
        
        POVButton dpadUp = new POVButton(driverController, 0);
        POVButton dpadDown = new POVButton(driverController, 180);
        POVButton dpadLeft = new POVButton(driverController, 270);
        POVButton dpadRight = new POVButton(driverController, 90);

        // Simplified triggers - complex logic moved to state machines
        Trigger notManualMode = new Trigger(() -> !elevManiStateMachine.getCurrentState().equals("Manual")).debounce(0.1);

        // ===== DRIVE STATE BINDINGS =====
        notManualMode.and(aButton).onTrue(Commands.runOnce(() -> 
            driveStateMachine.requestState(DriveStateMachine.DriveState.Teleop)));
        
        notManualMode.and(bButton).onTrue(Commands.runOnce(() -> 
            driveStateMachine.requestState(DriveStateMachine.DriveState.ReefRelative)));
        
        notManualMode.and(yButton).onTrue(Commands.runOnce(() -> 
            driveStateMachine.requestState(DriveStateMachine.DriveState.ProcessorRelative)));

        // ===== ELEVATOR/MANIPULATOR STATE BINDINGS =====
        notManualMode.and(xButton).onTrue(Commands.runOnce(() -> 
            elevManiStateMachine.requestState("L4Score")));
        
        notManualMode.and(dpadUp).onTrue(Commands.runOnce(() -> 
            elevManiStateMachine.requestState("AlgaeL3")));
        
        notManualMode.and(dpadDown).onTrue(Commands.runOnce(() -> 
            elevManiStateMachine.requestState("AlgaeL2")));

        // ===== SCORE SIDE SELECTION =====
        notManualMode.and(leftBumper).onTrue(Commands.runOnce(() -> 
            driveStateMachine.setAlignmentSide(false))); // Left side
        
        notManualMode.and(rightBumper).onTrue(Commands.runOnce(() -> 
            driveStateMachine.setAlignmentSide(true)));  // Right side

        // ===== MANUAL MODE TOGGLE =====
        leftStick.and(startButton).toggleOnTrue(Commands.runOnce(() -> 
            elevManiStateMachine.requestState("Manual")));

        // ===== MANUAL CONTROL BINDINGS =====
        Trigger manualMode = new Trigger(() -> elevManiStateMachine.getCurrentState().equals("Manual"));
        
        manualMode.and(yButton).onTrue(Commands.runOnce(() -> 
            elevator.incrementElevatorSetpoint(0.025))); // Elevator up
        
        manualMode.and(aButton).onTrue(Commands.runOnce(() -> 
            elevator.incrementElevatorSetpoint(-0.025))); // Elevator down
        
        manualMode.and(xButton).onTrue(Commands.runOnce(() -> 
            diffArm.incrementExtensionSetpoint(5))); // Arm out
        
        manualMode.and(bButton).onTrue(Commands.runOnce(() -> 
            diffArm.incrementExtensionSetpoint(-5))); // Arm in
        
        manualMode.and(leftBumper).onTrue(Commands.runOnce(() -> 
            diffArm.incrementRotationSetpoint(5))); // Rotate out
        
        manualMode.and(rightBumper).onTrue(Commands.runOnce(() -> 
            diffArm.incrementRotationSetpoint(-5))); // Rotate in

        // ===== INTAKE CONTROL =====
        dpadRight.onTrue(Commands.runOnce(() -> manipulator.runIntake(-0.9)))
                 .onFalse(Commands.runOnce(() -> manipulator.runIntake(0)));

        // ===== ALIGNMENT CONTROL =====
        leftTrigger.onTrue(Commands.runOnce(() -> 
            driveStateMachine.requestState(DriveStateMachine.DriveState.ReefAlign)))
                  .onFalse(Commands.runOnce(() -> 
            driveStateMachine.requestState(DriveStateMachine.DriveState.ReefRelative)));

        // ===== UTILITY CONTROLS =====
        rightStick.and(dpadLeft).onTrue(new InstantCommand(() -> 
            climber.setServoOpen())); // Servo open
        
        rightStick.and(dpadRight).onTrue(new InstantCommand(() -> 
            drive.zeroHeading())); // Zero heading

        // ===== EMERGENCY CONTROLS =====
        backButton.onTrue(Commands.runOnce(() -> {
            driveStateMachine.emergencyStop();
            elevManiStateMachine.emergencyReturnToSafe();
        }));

        startButton.and(backButton).onTrue(Commands.runOnce(() -> {
            driveStateMachine.reset();
            elevManiStateMachine.requestState("SafeCoralTravel");
        }));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Get the state machine coordinator for external access
     */
    public StateMachineCoordinator getStateMachineCoordinator() {
        return stateMachineCoordinator;
    }

    /**
     * Get the elevator/manipulator state machine for external access
     */
    public ElevatorManipulatorStateMachine getElevManiStateMachine() {
        return elevManiStateMachine;
    }

    /**
     * Get the drive state machine for external access
     */
    public DriveStateMachine getDriveStateMachine() {
        return driveStateMachine;
    }
}