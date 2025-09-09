// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import org.littletonrobotics.urcl.URCL;
import au.grapplerobotics.CanBridge;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;
    
    // Hardware subsystems
    private final LEDUtility ledUtility = new LEDUtility(0);
    private final XboxController driverController = new XboxController(0);
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(driveSubsystem);
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final ManipulatorSubsystem manipulatorSubsystem = new ManipulatorSubsystem();
    private final DifferentialSubsystem differentialSubsystem = new DifferentialSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    
    // State machines
    private ElevatorManipulatorStateMachine elevManiStateMachine;
    private DriveStateMachine driveStateMachine;
    private StateMachineCoordinator stateMachineCoordinator;

    public Robot() {
        CanBridge.runTCP();
    }

    /**
     * This function is run when the robot is first started up.
     */
    @Override
    public void robotInit() {
        // Initialize state machines
        elevManiStateMachine = new ElevatorManipulatorStateMachine(
            differentialSubsystem, elevatorSubsystem, manipulatorSubsystem);
        
        driveStateMachine = new DriveStateMachine(
            driveSubsystem, poseEstimator, driverController);
        
        stateMachineCoordinator = new StateMachineCoordinator(
            elevManiStateMachine, driveStateMachine);

        // Initialize robot container with all subsystems
        robotContainer = new RobotContainer(
            ledUtility,
            driveSubsystem,
            poseEstimator,
            elevatorSubsystem,
            manipulatorSubsystem,
            differentialSubsystem,
            climbSubsystem,
            elevManiStateMachine,
            driveStateMachine,
            stateMachineCoordinator,
            driverController
        );

        // Start data logging
        DataLogManager.start();
        URCL.start(DataLogManager.getLog());
    }

    /**
     * This function is called every 20 ms, no matter the mode.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        stateMachineCoordinator.setAutoMode(false);
        driveStateMachine.requestState(DriveStateMachine.DriveState.Teleop);
    }

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        stateMachineCoordinator.setAutoMode(true);
        
        // Reset to safe starting position
        elevManiStateMachine.requestState("SafeCoralTravel");
        driveStateMachine.requestState(DriveStateMachine.DriveState.FollowPath);
        
        // Get and schedule autonomous command
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        stateMachineCoordinator.setAutoMode(false);
        
        // Cancel autonomous command if still running
        if (autonomousCommand != null && autonomousCommand.isScheduled()) {
            autonomousCommand.cancel();
        }
        
        // Reset to safe teleop states
        elevManiStateMachine.requestState("SafeCoralTravel");
        driveStateMachine.requestState(DriveStateMachine.DriveState.Teleop);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        
        // Reset state machines for testing
        driveStateMachine.reset();
        elevManiStateMachine.requestState("SafeCoralTravel");
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
    
    /** Emergency stop handler - can be called from anywhere */
    public void emergencyStop() {
        driveStateMachine.emergencyStop();
        elevManiStateMachine.requestState("SafeCoralTravel");
        DataLogManager.log("EMERGENCY STOP ACTIVATED");
    }
}