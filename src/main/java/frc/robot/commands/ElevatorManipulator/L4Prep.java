package frc.robot.commands.ElevatorManipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;

public class L4Prep extends Command {

  private ManipulatorStateMachine manipulatorSm;
  private DifferentialSubsystem diffArm;
  private ElevatorSubsystem elevator;
  // private ManipulatorSubsystem manipulator;
  // private final Timer stepTimer = new Timer();
  // private int step = 0;
  private boolean atPosition = false;

  private double elevatorPos = 1.72;
  private double diffExt = 140;
  private double diffRot = 235;
  private double diffExt1 = 220;

  public L4Prep(
      ManipulatorStateMachine m_state, DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator) {

    manipulatorSm = m_state;
    diffArm = m_diff;
    elevator = m_elevator;
    // manipulator = m_manipulator;

    addRequirements(m_diff, elevator);
  }

  @Override
  public void initialize() {

    diffArm.setExtensionSetpoint(230);
    elevator.setElevatorSetpoint(elevatorPos);
    diffArm.setRotationSetpoint(diffRot);
  }

  @Override
  public void execute() {

    if (diffArm.atRotationSetpoint() && elevator.atPosition() && diffArm.atExtenstionSetpoint()) {
      atPosition = true;
    }

    // safety code if it takes too long extend out out
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atPosition;
  }
}
