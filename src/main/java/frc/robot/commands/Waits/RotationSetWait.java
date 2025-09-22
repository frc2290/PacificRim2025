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
/** Sets the rotation setpoint and waits until the differential arm matches it. */
public class RotationSetWait extends Command {
  private DifferentialSubsystem diff;
  private double setpoint;

  /** Creates a new ExtensionSetWait. */
  public RotationSetWait(DifferentialSubsystem _diff, double _setpoint) {
    diff = _diff;
    setpoint = _setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    diff.setRotationSetpoint(setpoint);
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
    return diff.atRotationSetpoint();
  }
}
