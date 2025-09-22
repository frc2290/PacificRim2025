// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

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
