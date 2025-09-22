// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
