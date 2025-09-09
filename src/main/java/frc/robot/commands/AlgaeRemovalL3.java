// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.ArmStateManager;
import frc.robot.subsystems.ArmStateManager.ElevatorManipulatorState;
import frc.robot.subsystems.DriveStateManager;
import frc.robot.subsystems.DriveStateManager.DriveState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeRemovalL3 extends Command {
  private ManipulatorSubsystem manip;
  private ArmStateManager arm;
  private DriveStateManager drive;
  private boolean level2;

  /** Creates a new AlgaeRemoval. */
    public AlgaeRemovalL3(ManipulatorSubsystem m_manip, ArmStateManager arm, DriveStateManager drive, boolean _level2) {
        manip = m_manip;
        this.arm = arm;
        this.drive = drive;
        level2 = _level2;
        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements(manip);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.setElevManiGoal(ElevatorManipulatorState.AlgaeL3);
        drive.setGoalDriveCommand(DriveState.Teleop).schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        manip.intake(-1.0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        manip.intake(0);
        arm.setElevManiGoal(ElevatorManipulatorState.IntakeCoral);
        drive.setGoalDriveCommand(DriveState.CoralStation).schedule();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
