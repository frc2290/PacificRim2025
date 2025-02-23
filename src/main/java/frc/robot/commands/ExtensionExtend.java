// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DifferentialArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ExtensionExtend extends Command {
  
  private final DifferentialArmSubsystem differentialArmSubsystem;

  private double power = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExtensionExtend(DifferentialArmSubsystem m_differentialArm, double m_power) {
    differentialArmSubsystem = m_differentialArm;
    power = m_power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(differentialArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    differentialArmSubsystem.Extend(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    differentialArmSubsystem.Extend(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
