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
import frc.robot.subsystems.ManipulatorSubsystem;

public class IntakeAlgae extends Command {
  private ManipulatorSubsystem manipulator;

  private Timer currentTimer = new Timer();
  private Timer delayTimer = new Timer();

  /** Creates a new IntakeOn. */
  public IntakeAlgae(ManipulatorSubsystem m_manip) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentTimer.reset();
    delayTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    manipulator.intake(-0.9);
    if (manipulator.getOutputCurrent() > 30) {
      if (!delayTimer.isRunning()) {
        delayTimer.restart();
      } else if (delayTimer.hasElapsed(1) && delayTimer.isRunning()) {
        delayTimer.stop();
        currentTimer.restart();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentTimer.stop();
    delayTimer.stop();
    if (!interrupted) {
      manipulator.intake(-0.5);
      manipulator.setAlgae(true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentTimer.hasElapsed(0.5);
  }
}
