package frc.robot.commands.ElevatorManipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DifferentialArm;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Moves the differential arm and elevator back to the safe travel pose using a sequenced approach
 * so the mechanisms never collide.
 */
public class SafeTravelSequential extends Command {
  private final DifferentialSubsystem diffArm;
  private final ElevatorSubsystem elevator;
  private int step = 0;
  private final Timer stepTimer = new Timer();

  /**
   * Creates a command that safely collapses the manipulator by stepping through extension,
   * rotation, then elevator moves with timeouts at each stage.
   */
  public SafeTravelSequential(DifferentialSubsystem diffArm, ElevatorSubsystem elevator) {
    this.diffArm = diffArm;
    this.elevator = elevator;
    addRequirements(diffArm, elevator);
  }

  @Override
  public void initialize() {
    step = 0;
    stepTimer.restart();
    System.out.println("To CoralSafeTravel INITIALIZING");
  }

  @Override
  public void execute() {
    switch (step) {
      case 0:
        // First extend to clear the elevator while still facing the goal.
        diffArm.setExtensionSetpoint(230);
        if (diffArm.atExtenstionSetpoint() || stepTimer.hasElapsed(2.0)) {
          step = 1;
          stepTimer.restart();
          System.out.println("To CoralSafeTravel 1/3");
        }

        break;

      case 1:
        // Swing the arm back over the bumper after we have clearance.
        diffArm.setRotationSetpoint(DifferentialArm.transportRotationSetpoint);
        if (diffArm.atRotationSetpoint() || stepTimer.hasElapsed(2.0)) {
          step = 2;
          stepTimer.restart();
          System.out.println("To CoralSafeTravel 2/3");
        }

        break;

      case 2:
        // Finally retract and lower to the safe transport pose.
        diffArm.setExtensionSetpoint(DifferentialArm.transportExtensionSetpoint);
        elevator.setElevatorSetpoint(0.2);
        if ((diffArm.atExtenstionSetpoint() && elevator.atPosition())
            || stepTimer.hasElapsed(2.0)) {
          step = 3;
          stepTimer.restart();
          System.out.println("To CoralSafeTravel 3/3");
        }

        break;
    }
  }

  @Override
  public boolean isFinished() {
    // Once the third step completes the system is safe to drive around.
    return step == 3;
  }

  @Override
  public void end(boolean interrupted) {
    // Cleanup if needed
  }
}
