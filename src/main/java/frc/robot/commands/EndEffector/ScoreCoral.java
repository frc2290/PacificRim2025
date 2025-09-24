// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorSubsystem;

/** Runs the manipulator roller in reverse to score a coral into the reef. */
public class ScoreCoral extends Command {

  private static final double kScoreDurationSeconds = 1.0;

  private final ManipulatorStateMachine manipulatorSM;
  private final ManipulatorSubsystem manipulator;
  private final Timer ejectTimer = new Timer();
  private final double ejectPower;

  private boolean spinning = false;

  /**
   * Creates a command that waits for the manipulator state machine to report ready, then runs the
   * roller to deposit the coral.
   */
  public ScoreCoral(
      ManipulatorStateMachine manipulatorStateMachine, ManipulatorSubsystem manipulatorSubsystem) {
    this(manipulatorStateMachine, manipulatorSubsystem, 1.0);
  }

  /** Creates a coral scoring command that uses the supplied roller power. */
  protected ScoreCoral(
      ManipulatorStateMachine manipulatorStateMachine,
      ManipulatorSubsystem manipulatorSubsystem,
      double power) {
    manipulatorSM = manipulatorStateMachine;
    manipulator = manipulatorSubsystem;
    ejectPower = power;

    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.resetMotorPos();
    ejectTimer.stop();
    ejectTimer.reset();
    spinning = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean readyToSpin = manipulatorSM.atGoalState() && manipulatorSM.atDrivePose() && manipulatorSM.scoreNow();

    if (readyToSpin && !spinning) {
      spinning = true;
      ejectTimer.restart();
    }

    manipulator.intake(spinning ? ejectPower : 0);
  }

  @Override
  public void end(boolean interrupted) {
    ejectTimer.stop();
    manipulator.intake(0);
    if (!interrupted) {
      // Clear the coral/algae flags so the subsystem knows it is empty again.
      manipulator.setCoral(false);
      manipulator.setAlgae(false);
    }
    manipulatorSM.atGoalState(false);
    spinning = false;
  }

  @Override
  public boolean isFinished() {
    // Run the roller for a fixed amount of time to guarantee the coral is released.
    return spinning && ejectTimer.hasElapsed(kScoreDurationSeconds);
  }
}
