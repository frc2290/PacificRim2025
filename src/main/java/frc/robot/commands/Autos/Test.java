package frc.robot.commands.Autos;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveAutoStep;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.DriveState;
import frc.utils.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Test extends SequentialCommandGroup {
  /** Creates a new SPSourceSide. */
  public Test(PoseEstimatorSubsystem poseEst, StateSubsystem stateSubsystem) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("RightCoral1");
      // stateSubsystem.setRotationLock(false);

      Command resetPose = new InstantCommand(() -> poseEst.setCurrentPose(path.getStartingHolonomicPose().get()));
      // Load the path you want to follow using its name in the GUI
      addCommands(stateSubsystem.setDriveStateCommand(DriveState.Auto), resetPose, new SwerveAutoStep(path, poseEst));
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      Commands.none();
    }
  }
}
