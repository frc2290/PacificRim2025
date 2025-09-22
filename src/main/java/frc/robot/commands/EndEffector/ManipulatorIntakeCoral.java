package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

/** Spins the manipulator intake until a coral is captured and then cleared from the sensor. */
public class ManipulatorIntakeCoral extends Command {

  private ManipulatorSubsystem manipulator;

  /** Not currently used, but retained in case time-based tweaks are needed. */
  private Timer currentTimer = new Timer();

  /** Optional dwell timer once coral is detected. */
  private Timer delayTimer = new Timer();

  private boolean sawCoral = false;
  private boolean finished = false;

  /**
   * Creates a command that spins the intake until coral is captured, then stops the roller once the
   * beam break clears.
   */
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

    // Run the intake until we see a coral break the beam.
    manipulator.intake(0.55);
    if (!sawCoral && manipulator.seesCoral()) {
      sawCoral = true;
    }
    if (sawCoral) {
      // Once the coral clears the sensor we know it is fully inside the manipulator.
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
      // Stop the intake once the coral is secure and flag the subsystem state.
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
