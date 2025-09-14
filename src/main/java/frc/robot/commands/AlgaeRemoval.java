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
public class AlgaeRemoval extends Command {
  private ManipulatorSubsystem manip;
  private StateSubsystem state;
  private boolean level2;

  /** Creates a new AlgaeRemoval. */
  public AlgaeRemoval(ManipulatorSubsystem m_manip, StateSubsystem m_state, boolean _level2) {
    manip = m_manip;
    state = m_state;
    level2 = _level2;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(manip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state.setGoal(PositionState.AlgaeL2Position);
    state.setDriveState(DriveState.Teleop);
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
    state.setGoal(PositionState.IntakePosition);
    state.setDriveState(DriveState.CoralStation);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
