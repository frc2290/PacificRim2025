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

import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorSubsystem;

/** Runs the manipulator roller in reverse to score a coral into the reef. */
public class ScoreCoralL4 extends ScoreCoral {

  private static final double kL4EjectPower = -1.0;

  /**
   * Creates a command that waits for the manipulator state machine to report ready, then runs the
   * roller to deposit the coral at L4.
   */
  public ScoreCoralL4(
      ManipulatorStateMachine manipulatorStateMachine, ManipulatorSubsystem manipulatorSubsystem) {
    super(manipulatorStateMachine, manipulatorSubsystem, kL4EjectPower);
  }
}
