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

/** Spins the manipulator roller to dump algae into a processor or barge. */
public class ScoreAlgae extends Command {

  private static final double kEjectDurationSeconds = 1;

  private final ManipulatorStateMachine manipulatorSM;
  private final ManipulatorSubsystem manipulator;
  private final Timer ejectTimer = new Timer();
  private final double ejectPower;

  private boolean spinning = false;

  /**
   * Creates a command that waits for the manipulator to reach its goal, then spins the roller to
   * eject algae.
   */
  public ScoreAlgae(
      ManipulatorStateMachine manipulatorStateMachine, ManipulatorSubsystem manipulatorSubsystem) {
    this(manipulatorStateMachine, manipulatorSubsystem, 1.0);
  }

  /**
   * Creates a command that waits for the manipulator to reach its goal, then spins the roller to
   * eject algae using the supplied power.
   */
  public ScoreAlgae(
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
    ejectTimer.stop();
    ejectTimer.reset();
    spinning = false;
  }

  @Override
  public void execute() {
    boolean readyToSpin = manipulatorSM.atGoalState() && manipulatorSM.scoreNow();

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
      manipulator.setCoral(false);
      manipulator.setAlgae(false);
    }
    manipulatorSM.atGoalState(false);
    spinning = false;
  }

  @Override
  public boolean isFinished() {
    return spinning && ejectTimer.hasElapsed(kEjectDurationSeconds);
  }
}
