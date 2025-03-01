// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.State;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L3Position extends Command {
  private double elevatorPos = 1.05;
  private double diffExt = 25;
  private double diffRot = (220 - 40);

  private ElevatorSubsystem elevator;
  private DifferentialSubsystem diff;
  private StateSubsystem state;

  private boolean moved_ext = false;
  private boolean moved_rot = false;
  private boolean moved_elev = false;
  /** Creates a new TravelPosition. */
  public L3Position(DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator, StateSubsystem m_state) {
    elevator = m_elevator;
    diff = m_diff;
    state = m_state;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(m_diff, m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moved_ext = false;
    moved_rot = false;
    moved_elev = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setElevatorSetpoint(elevatorPos);
    if (elevator.atPosition() && moved_elev) {
      diff.setRotationSetpoint(diffRot);
      if (diff.atRotationSetpoint() && moved_rot) {
        diff.setExtensionSetpoint(diffExt);
        moved_ext = true;
      }
      moved_rot = true;
    }
    moved_elev = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    state.setCurrentState(State.L3Position);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (moved_ext && moved_rot && moved_elev);
    //return false;
  }
}
