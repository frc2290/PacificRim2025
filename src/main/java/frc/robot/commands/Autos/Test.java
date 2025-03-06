package frc.robot.commands.Autos;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;

import frc.utils.LEDUtility;
import frc.utils.PoseEstimatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Test extends SequentialCommandGroup{
  /** Creates a new SPSourceSide. */
  public Test(PoseEstimatorSubsystem poseEst) {


    
    
    try{

      PathPlannerPath path = PathPlannerPath.fromPathFile("Drive");

      Command resetPose = new InstantCommand(() -> poseEst.setCurrentPose(path.getStartingHolonomicPose().get()));
      // Load the path you want to follow using its name in the GUI
      addCommands(resetPose, AutoBuilder.followPath(path));

      // Create a path following command using AutoBuilder. This will also trigger event markers.
      AutoBuilder.followPath(path);

  } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      Commands.none();
  }

    
  
  }
}
