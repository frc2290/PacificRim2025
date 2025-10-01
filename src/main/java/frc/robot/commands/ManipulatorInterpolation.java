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
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ManipulatorStateMachine;

public class ManipulatorInterpolation extends Command {

  private DifferentialSubsystem diffArm;
  private ManipulatorStateMachine manipulatorSM;

  // does not have any requirmennts, use carfuly inside other commands
  public ManipulatorInterpolation(
      DifferentialSubsystem diffArm, ManipulatorStateMachine manipulatorSM) {
    this.diffArm = diffArm;
    this.manipulatorSM = manipulatorSM;

    addRequirements(diffArm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    interpolation();
  }

  private void interpolation() {

    /** Diff Arm Interpolation */
    if (diffArm.hasLaserCanDistance()) {
      diffArm.setExtensionSetpoint(diffArm.l4ExtensionInterpolate());
      diffArm.setRotationSetpoint(diffArm.l4RotationInterpolate());
    }

    if (diffArm.atExtenstionSetpoint() && diffArm.atRotationSetpoint()) {
      manipulatorSM.setreadyToScore(true);
    } else {
      manipulatorSM.setreadyToScore(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
