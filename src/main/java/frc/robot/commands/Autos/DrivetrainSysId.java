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
package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
/** Command group that runs the four System Identification tests for the drivetrain. */
public class DrivetrainSysId extends SequentialCommandGroup {
  /** Creates a new DrivetrainSysId. */
  public DrivetrainSysId(DriveSubsystem drive) {
    // Run forward and reverse quasistatic tests followed by dynamic tests. The boolean parameter
    // enables logging so the data can be exported into SysId later.
    addCommands(
        drive.getQuasistaticForward(true),
        drive.getQuasistaticReverse(true),
        drive.getDynamicForward(true),
        drive.getDynamicReverse(true));
  }
}
