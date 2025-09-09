// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Waits;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmStateManager;
import frc.robot.subsystems.ArmStateManager.ElevatorManipulatorState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetGoalWait extends Command {
  private ArmStateManager arm;
  private ElevatorManipulatorState goal;

  /** Creates a new SetGoalWait. */
  public SetGoalWait(ArmStateManager arm, ElevatorManipulatorState goal) {
    this.arm = arm;
    this.goal = goal;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setElevManiGoal(goal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.atElevManiGoal();
  }
}
