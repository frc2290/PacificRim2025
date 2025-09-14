// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.SwerveAutoStep;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.DriveState;
import frc.robot.subsystems.StateSubsystem.PositionState;
import frc.utils.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Right2Coral extends SequentialCommandGroup {
  /** Creates a new Right2Coral. */
  public Right2Coral(
      DifferentialSubsystem diff,
      PoseEstimatorSubsystem poseEst,
      StateSubsystem stateSubsystem,
      ManipulatorSubsystem manipulator) {
    try {
      // stateSubsystem.setRotationLock(false);
      // Pull in path from start location to reef
      PathPlannerPath startToReef = PathPlannerPath.fromPathFile("RightCoral1");
      PathPlannerPath reefToFeed = PathPlannerPath.fromPathFile("RightCoral1ToFeeder");
      PathPlannerPath feedToReef2 = PathPlannerPath.fromPathFile("FeederToRightCoral2");

      // Create a reset pose command to set starting location (may remove in future)
      Command resetPose =
          new InstantCommand(
              () -> poseEst.setCurrentPose(startToReef.getStartingHolonomicPose().get()));

      // Set drive to auto (have to do this for every auto)
      Command driveSetAuto = stateSubsystem.setDriveStateCommand(DriveState.Auto);

      // Create a parallel group to move to the reef and get in scoring position at the same time
      Command followPath1 = new SwerveAutoStep(startToReef, poseEst);
      Command moveToReef =
          new ParallelCommandGroup(
              followPath1, stateSubsystem.setGoalCommand(PositionState.L4Position, true));

      Command followPath2 = new SwerveAutoStep(reefToFeed, poseEst);
      Command moveToFeeder =
          new ParallelCommandGroup(followPath2, new IntakeCoral(manipulator, stateSubsystem));

      Command followPath3 = new SwerveAutoStep(feedToReef2, poseEst);
      Command moveToReef2 =
          new ParallelCommandGroup(
              followPath3, stateSubsystem.setGoalCommand(PositionState.L4Position, true));

      // Set drive to teleop (have to do this for every auto)
      Command driveSetTeleop = stateSubsystem.setDriveStateCommand(DriveState.Teleop);

      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      // First reset position, move to reef and get in score position, lastly score the coral
      addCommands(
          resetPose,
          driveSetAuto,
          moveToReef,
          // new WaitCommand(0.5),
          new ScoreCoral(manipulator, diff, stateSubsystem, poseEst),
          moveToFeeder,
          new WaitCommand(0.5),
          moveToReef2,
          // new WaitCommand(0.5),
          new ScoreCoral(manipulator, diff, stateSubsystem, poseEst),
          driveSetTeleop);
    } catch (Exception e) {
      System.out.println("BROKENNNNNNNNNNNNNNNNN");
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      Commands.none();
    }
  }
}
