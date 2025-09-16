package frc.robot.commands.ElevatorManipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DifferentialArm;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SafeTravelSequential extends Command {
  private final DifferentialSubsystem diffArm;
  private final ElevatorSubsystem elevator;
  private int step = 0;
  private final Timer stepTimer = new Timer();

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
        diffArm.setExtensionSetpoint(230);
        if (diffArm.atExtenstionSetpoint() || stepTimer.hasElapsed(2.0)) {
          step = 1;
          stepTimer.restart();
          System.out.println("To CoralSafeTravel 1/3");
        }

        break;

      case 1:
        diffArm.setRotationSetpoint(DifferentialArm.transportRotationSetpoint);
        if (diffArm.atRotationSetpoint() || stepTimer.hasElapsed(2.0)) {
          step = 2;
          stepTimer.restart();
          System.out.println("To CoralSafeTravel 2/3");
        }

        break;

      case 2:
        diffArm.setExtensionSetpoint(DifferentialArm.transportExtensionSetpoint);
        if (diffArm.atExtenstionSetpoint() || stepTimer.hasElapsed(2.0)) {
          step = 3;
          stepTimer.restart();
          System.out.println("To CoralSafeTravel 3/3");
        }

        break;
    }
  }

  @Override
  public boolean isFinished() {
    return step == 3;
  }

  @Override
  public void end(boolean interrupted) {
    // Cleanup if needed
  }
}
