package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ManipulatorIntakeCoral extends Command {

  private ManipulatorSubsystem manipulator;

  private Timer currentTimer = new Timer();
  private Timer delayTimer = new Timer();

  private boolean sawCoral = false;
  private boolean finished = false;

  public ManipulatorIntakeCoral(ManipulatorSubsystem m_manipulator) {
    manipulator = m_manipulator;

    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    currentTimer.reset();
    delayTimer.reset();
    sawCoral = false;
    finished = false;
  }

  @Override
  public void execute() {

    manipulator.intake(0.55);
    if (!sawCoral && manipulator.seesCoral()) {
      sawCoral = true;
    }
    if (sawCoral) {
      if (!manipulator.seesCoral()) {
        finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentTimer.stop();
    delayTimer.stop();
    if (!interrupted) {
      manipulator.intake(0);
      manipulator.setCoral(true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
