package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ScoreCoral extends Command {

  ManipulatorStateMachine manipulatorSM;
  ManipulatorSubsystem manipulator;
  private Timer timer = new Timer();

  public ScoreCoral(ManipulatorStateMachine m_manipulatorSM, ManipulatorSubsystem m_manipulator) {

    manipulatorSM = m_manipulatorSM;
    manipulator = m_manipulator;
    addRequirements(manipulator);
  }

  @Override
  public void initialize() {
    manipulator.resetMotorPos();
    timer.reset();
    System.out.println("Starting Score Coral");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if ((pose.atTargetPose(diff.hasLaserCanDistance()) && state.atCurrentState()) ||
    // !state.getRotationLock()) {
    if (manipulatorSM.atGoalState() && manipulatorSM.canScore()) {
      manipulator.intake(1);
      if (!timer.isRunning()) {
        timer.restart();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    System.out.println("Time to score: " + timer.get());
    manipulator.intake(0);
    if (!interrupted) {
      manipulator.setCoral(false);
      manipulator.setAlgae(false);
      // manipulatorSM.atGoalState(false);
    }
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1);
    // return manipulator.getMotorPos() > 200;// || !manipulator.hasCoral();
  }
}
