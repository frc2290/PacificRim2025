// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
public class Right3Coral extends SequentialCommandGroup {
  /** Creates a new Right3Coral. */
  public Right3Coral(
      DifferentialSubsystem diff,
      PoseEstimatorSubsystem poseEst,
      StateSubsystem stateSubsystem,
      ManipulatorSubsystem manipulator) {
    try {
      Timer timer = new Timer();
      // Pull in path from start location to reef
      PathPlannerPath startToReef = PathPlannerPath.fromPathFile("RightCoral1");
      PathPlannerPath reefToFeed = PathPlannerPath.fromPathFile("RightCoral1ToFeeder");
      PathPlannerPath feedToReef2 = PathPlannerPath.fromPathFile("FeederToRightCoral2");
      PathPlannerPath reef2toFeed = PathPlannerPath.fromPathFile("RightCoral2ToFeeder");
      PathPlannerPath feedToReef3 = PathPlannerPath.fromPathFile("FeederToRightCoral3");
      PathPlannerPath reef3ToFeed = PathPlannerPath.fromPathFile("RightCoral3ToFeeder");

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
      // Command scoreCoral1 = new SwerveAutoScore(VisionConstants.leftBranches.get(2), manipulator,
      // stateSubsystem, poseEst);

      Command followPath2 = new SwerveAutoStep(reefToFeed, poseEst);
      Command moveToFeeder = new ParallelCommandGroup(followPath2);

      Command followPath3 = new SwerveAutoStep(feedToReef2, poseEst);
      Command moveToReef2 =
          new ParallelCommandGroup(
              followPath3,
              stateSubsystem
                  .setGoalCommand(PositionState.L4Position, true)
                  .beforeStarting(new WaitCommand(0.25)));
      // Command scoreCoral2 = new SwerveAutoScore(VisionConstants.rightBranches.get(3),
      // manipulator, stateSubsystem, poseEst);

      Command followPath4 = new SwerveAutoStep(reef2toFeed, poseEst);
      Command moveToFeeder2 = new ParallelCommandGroup(followPath4);

      Command followPath5 = new SwerveAutoStep(feedToReef3, poseEst);
      Command moveToReef3 =
          new ParallelCommandGroup(
              followPath5,
              stateSubsystem
                  .setGoalCommand(PositionState.L4Position, true)
                  .beforeStarting(new WaitCommand(0.25)));
      // Command scoreCoral3 = new SwerveAutoScore(VisionConstants.leftBranches.get(3), manipulator,
      // stateSubsystem, poseEst);

      Command followPath6 = new SwerveAutoStep(reef3ToFeed, poseEst);

      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      // First reset position, move to reef and get in score position, lastly score the coral
      addCommands(
          Commands.runOnce(() -> timer.restart()),
          resetPose,
          driveSetAuto,
          moveToReef,
          new ScoreCoral(manipulator, diff, stateSubsystem, poseEst),
          moveToFeeder,
          new WaitCommand(1.5),
          moveToReef2,
          new ScoreCoral(manipulator, diff, stateSubsystem, poseEst),
          moveToFeeder2,
          new WaitCommand(1.5),
          moveToReef3,
          new ScoreCoral(manipulator, diff, stateSubsystem, poseEst),
          Commands.runOnce(
              () -> {
                timer.stop();
                System.out.println("Right3Coral Time: " + timer.get());
              }) // ,
          // followPath6
          // driveSetTeleop,
          );
    } catch (Exception e) {
      System.out.println("BROKENNNNNNNNNNNNNNNNN");
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      Commands.none();
    }
  }
}
