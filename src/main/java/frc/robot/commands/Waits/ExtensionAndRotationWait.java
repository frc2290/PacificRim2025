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
package frc.robot.commands.Waits;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Sets both the differential arm extension and rotation setpoints and waits until they are reached.
 */
public class ExtensionAndRotationWait extends Command {
  private DifferentialSubsystem diff;
  private double extSetpoint;
  private double rotSetpoint;

  /** Creates a new ExtensionSetWait. */
  public ExtensionAndRotationWait(
      DifferentialSubsystem _diff, double _extSetpoint, double _rotSetpoint) {
    diff = _diff;
    extSetpoint = _extSetpoint;
    rotSetpoint = _rotSetpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    diff.setExtensionSetpoint(extSetpoint);
    diff.setRotationSetpoint(rotSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Only finish once both setpoints have been reached.
    return diff.atExtenstionSetpoint() && diff.atRotationSetpoint();
  }
}
