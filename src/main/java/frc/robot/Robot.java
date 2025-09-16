// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.io.DifferentialArmIO;
import frc.robot.io.DifferentialArmIOReal;
import frc.robot.io.DifferentialArmIOSim;
import frc.robot.io.DriveIO;
import frc.robot.io.DriveIOReal;
import frc.robot.io.DriveIOSim;
import frc.robot.io.ElevatorIO;
import frc.robot.io.ElevatorIOReal;
import frc.robot.io.ElevatorIOSim;
import frc.robot.io.ManipulatorIO;
import frc.robot.io.ManipulatorIOReal;
import frc.robot.io.ManipulatorIOSim;
import frc.robot.io.VisionIO;
import frc.robot.io.VisionIOReal;
import frc.robot.io.VisionIOSim;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.DriveStateMachine;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateMachineCoardinator;
import frc.robot.subsystems.StateMachineCoardinator.RobotState;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;
import frc.utils.LEDUtility;
import frc.utils.PoseEstimatorSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final LEDUtility m_ledUtility = new LEDUtility(0);
  private final XboxController m_driver = new XboxController(0);
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive;
  private final PoseEstimatorSubsystem m_poseEstimator;
  private final ElevatorSubsystem m_elevator;
  private final ManipulatorSubsystem m_manipulator;
  private final DifferentialSubsystem m_DiffArm;
  private final ClimbSubsystem m_climber = new ClimbSubsystem();
  private final DriveStateMachine m_driveStateMachine;
  private final ManipulatorStateMachine m_manipulatorStateMachine;
  private final StateMachineCoardinator m_coordinator;

  private final FlytLogger simDash = new FlytLogger("Simulation");
  private double totalCurrentDraw;
  private double loadedBatteryVoltage;
  private double m_simulatedBatteryVoltage;

  public Robot() {
    if (RobotBase.isReal()) {
      CanBridge.runTCP();
    }

    DriveIO driveIO = RobotBase.isSimulation() ? new DriveIOSim() : new DriveIOReal();
    m_robotDrive = new DriveSubsystem(driveIO);
    VisionIO visionIO =
        RobotBase.isSimulation() ? new VisionIOSim(m_robotDrive) : new VisionIOReal();
    m_poseEstimator = new PoseEstimatorSubsystem(m_robotDrive, visionIO);

    ElevatorIO elevatorIO = RobotBase.isSimulation() ? new ElevatorIOSim() : new ElevatorIOReal();
    m_elevator = new ElevatorSubsystem(elevatorIO);

    ManipulatorIO manipulatorIO =
        RobotBase.isSimulation() ? new ManipulatorIOSim() : new ManipulatorIOReal();
    m_manipulator = new ManipulatorSubsystem(manipulatorIO);

    DifferentialArmIO diffIO =
        RobotBase.isSimulation() ? new DifferentialArmIOSim() : new DifferentialArmIOReal();
    m_DiffArm = new DifferentialSubsystem(diffIO);

    m_driveStateMachine = new DriveStateMachine(m_robotDrive, m_poseEstimator, m_driver);
    m_manipulatorStateMachine =
        new ManipulatorStateMachine(m_DiffArm, m_elevator, m_manipulator, m_climber);
    m_coordinator = new StateMachineCoardinator(m_manipulatorStateMachine, m_driveStateMachine);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    if (RobotBase.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
      SimulatedArena.overrideInstance(new Arena2025Reefscape());
      SimulatedArena.getInstance().placeGamePiecesOnField();
      SimulatedArena.getInstance().addDriveTrainSimulation(m_robotDrive.getDriveSimulation());
    }

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
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
            m_coordinator,
            m_driver);
    DataLogManager.start();

    URCL.start();
    // Start URCL once, logging to the WPILib DataLog
    URCL.start(DataLogManager.getLog());

    if (RobotBase.isSimulation()) {
      SmartDashboard.putData("Field", m_robotDrive.getField());
      simDash.addDoublePublisher("Total Current", false, () -> totalCurrentDraw);
      simDash.addDoublePublisher("Loaded Voltage", false, () -> loadedBatteryVoltage);
      simDash.addDoublePublisher("Differential Current", false, m_DiffArm::getCurrentDraw);
      simDash.addDoublePublisher(
          "Differential Velocity", false, m_DiffArm::getMeasuredExtensionVelocity);
      simDash.addDoublePublisher(
          "Differential Rotation Velocity", false, m_DiffArm::getMeasuredRotationVelocity);
      simDash.addDoublePublisher("Elevator Current", false, m_elevator::getCurrentDraw);
      simDash.addDoublePublisher(
          "Elevator Velocity", false, m_elevator::getVelocityMetersPerSecond);
      simDash.addDoublePublisher("Manipulator Current", false, m_manipulator::getCurrentDraw);
      simDash.addDoublePublisher("Manipulator RPM", false, m_manipulator::getRPM);
    }
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  @Override
  public void simulationInit() {
    m_simulatedBatteryVoltage = SimulatedBattery.getBatteryVoltage().in(Volts);
    RoboRioSim.setVInVoltage(m_simulatedBatteryVoltage);
  }

  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();

    if (m_robotContainer != null) {
      m_robotContainer.simulationPeriodic();
    }

    Pose2d pose = m_robotDrive.getSimPose();
    m_poseEstimator.addVisionPoseMeasurement(pose, Timer.getFPGATimestamp());

    totalCurrentDraw = SimulatedBattery.getTotalCurrentDrawn().in(Amps);
    loadedBatteryVoltage = SimulatedBattery.getBatteryVoltage().in(Volts);
    m_simulatedBatteryVoltage = loadedBatteryVoltage;
    RoboRioSim.setVInVoltage(m_simulatedBatteryVoltage);
    simDash.update(Constants.debugMode);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_coordinator.robotDisabled(true);
    if (m_autonomousCommand != null && m_autonomousCommand.isScheduled()) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_coordinator.robotAuto(true);
    m_coordinator.robotDisabled(false);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_coordinator.robotAuto(false);
    m_coordinator.robotDisabled(false);
    m_coordinator.setRobotGoal(RobotState.START_POSITION);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
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
