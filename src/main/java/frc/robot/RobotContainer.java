// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Auto;
import frc.robot.commands.ElevatorMove;
import frc.robot.commands.ExtensionExtend;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.RollEndeffector;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.WristRotate;
import frc.robot.commands.Autos.Test;
import frc.robot.commands.Positions.IntakePosition;
import frc.robot.commands.Positions.L1Position;
import frc.robot.commands.Positions.L2Position;
import frc.robot.commands.Positions.L3Position;
import frc.robot.commands.Positions.L4Position;
import frc.robot.commands.Positions.TravelPosition;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.State;
import frc.utils.PoseEstimatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final PoseEstimatorSubsystem m_poseEstimator = new PoseEstimatorSubsystem(m_robotDrive);
    private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
    private final ManipulatorSubsystem m_endeffector = new ManipulatorSubsystem();
    private final DifferentialSubsystem m_DiffArm = new DifferentialSubsystem(m_endeffector);
    private final StateSubsystem m_state = new StateSubsystem(m_DiffArm, m_elevator, m_robotDrive);

	// The driver's controller
	XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

	// Auto Chooser for Dashboard
	SendableChooser<Command> auto_chooser = new SendableChooser<>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {// Configure the button bindings
		configureButtonBindings();

		// Build an auto chooser. This will use Commands.none() as the default option.
      auto_chooser.addOption("Drive", new Test(m_poseEstimator));
      auto_chooser.addOption("Driving", new Auto(m_robotDrive));
      SmartDashboard.putData(auto_chooser);

		// Configure default commands
		m_robotDrive.setDefaultCommand(
			// The left stick controls translation of the robot.
			// Turning is controlled by the X axis of the right stick.
			new RunCommand(
				() -> m_robotDrive.drive(
					-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
					-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
					-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
					true),
				m_robotDrive));
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
    // new JoystickButton(m_driverController, Button.kRightBumper.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));
    // new JoystickButton(m_driverController, Button.kLeftBumper.value)
    //     .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(m_state.setGoalCommand(State.L1Position));
        //.onTrue(new L1Position(m_DiffArm, m_elevator));
    new JoystickButton(m_driverController, Button.kB.value)
        .onTrue(m_state.setGoalCommand(State.L2Position));
        //.onTrue(new L2Position(m_DiffArm, m_elevator));
    new JoystickButton(m_driverController, Button.kY.value)
        .onTrue(m_state.setGoalCommand(State.L3Position));
        //.onTrue(new L3Position(m_DiffArm, m_elevator));
    new JoystickButton(m_driverController, Button.kX.value)
        .onTrue(m_state.setGoalCommand(State.L4Position));
        //.onTrue(new L4Position(m_DiffArm, m_elevator));
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .onTrue(new IntakeCoral(m_endeffector, m_state));
        //.onTrue(new IntakeCoral(m_endeffector, m_state));
        //.whileTrue(new RollEndeffector(m_endeffector, 0.5)); //intake
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new ScoreCoral(m_endeffector, m_state, m_robotDrive));
        //.whileTrue(new RollEndeffector(m_endeffector, -0.5)); //outtake

    new POVButton(m_driverController, 0)
        .onTrue(m_state.setGoalCommand(State.TravelPosition));
        //.onTrue(new TravelPosition(m_DiffArm, m_elevator));
    new POVButton(m_driverController, 180)
        .onTrue(m_state.setGoalCommand(State.IntakePosition));
        //.onTrue(new IntakePosition(m_DiffArm, m_elevator));
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