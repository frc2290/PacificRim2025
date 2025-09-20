package frc.robot.commands.ElevatorManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DifferentialArm;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;

/** Moves the elevator and differential arm to the intake position. */
public class IntakeCoral extends Command {

  private ManipulatorStateMachine manipulatorSm;
  private DifferentialSubsystem diffArm;
  private ElevatorSubsystem elevator;
  // private ManipulatorSubsystem manipulator;
  // private final Timer stepTimer = new Timer();
  // private int step = 0;
  private boolean atPosition = false;
  private boolean hasCoral = false;

  /**
   * Creates a command that positions the elevator and differential arm to accept a coral from the
   * human player.
   */
  public IntakeCoral(
      ManipulatorStateMachine m_state, DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator) {

    manipulatorSm = m_state;
    diffArm = m_diff;
    elevator = m_elevator;
    // manipulator = m_manipulator;

    addRequirements(m_diff, elevator);
  }

  @Override
  public void initialize() {
    // Command the mechanisms to the predefined intake pose.
    diffArm.setExtensionSetpoint(DifferentialArm.intakeExtensionSetpoint);
    diffArm.setRotationSetpoint(DifferentialArm.intakeRotationSetpoint);
    elevator.setElevatorSetpoint(Elevator.intakeSetpoint);
  }

  @Override
  public void execute() {

    // Track when both mechanisms reach their targets so we can signal the state machine.
    if (diffArm.atExtenstionSetpoint() && diffArm.atRotationSetpoint()) {
      atPosition = true;
    }

    hasCoral = manipulatorSm.getHasCoral();

    // Placeholder for safety fallback if we need to retract after a timeout.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Finish once the robot is in position and the manipulator has detected a coral.
    return atPosition && hasCoral;
  }
}
