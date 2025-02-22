package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DifferentialArm;
import frc.robot.subsystems.DifferentialArmSubsystem;


public class WristRotate extends Command{

      private final DifferentialArmSubsystem differentialArmSubsystem;

  private double power = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WristRotate(DifferentialArmSubsystem m_differentialArm, double m_power) {
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
    differentialArmSubsystem.Rotate(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    differentialArmSubsystem.Rotate(0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
    