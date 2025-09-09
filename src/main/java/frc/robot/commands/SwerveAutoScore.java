// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.ArmStateManager;
import frc.robot.subsystems.DriveStateManager;
import frc.utils.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveAutoScore extends ParallelCommandGroup {
  /** Creates a new SwerveAutoScore. */
  public SwerveAutoScore(Pose2d target, ManipulatorSubsystem manipulator, DifferentialSubsystem diff,
                         ArmStateManager arm, DriveStateManager drive, PoseEstimatorSubsystem pose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Command score = new ScoreCoral(manipulator, diff, arm, drive, pose);
    Command wait = new WaitCommand(0.25);
    Command waitAndScore = new SequentialCommandGroup(wait, score);
    Command align = pose.setTargetPoseCommand(target);

    addCommands(align, waitAndScore);
  }
}
