// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.DriveState;
import frc.robot.subsystems.StateSubsystem.PositionState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoral extends Command {
  private ManipulatorSubsystem manipulator;
  private StateSubsystem state;

  private int count = 0;

  /** Creates a new ScoreCoral. */
  public ScoreCoral(ManipulatorSubsystem m_manip, StateSubsystem m_state) {
    manipulator = m_manip;
    state = m_state;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    manipulator.resetMotorPos();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    manipulator.intake(0.75);
    count++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manipulator.intake(0);
    manipulator.setCoral(false);
    state.setGoal(PositionState.IntakePosition);
    //state.setDriveState(DriveState.CoralStation);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count > 20 && manipulator.getMotorPos() > 50;
  }
}
