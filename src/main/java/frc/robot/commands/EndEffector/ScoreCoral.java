// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorSubsystem;

/** Runs the manipulator roller in reverse to score a coral into the reef. */
public class ScoreCoral extends Command {

  ManipulatorStateMachine manipulatorSM;
  ManipulatorSubsystem manipulator;
  private Timer timer = new Timer();

  /**
   * Creates a command that waits for the manipulator state machine to report ready, then runs the
   * roller to deposit the coral.
   */
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
    if (manipulatorSM.atGoalState() && manipulatorSM.readyToScore() && manipulatorSM.scoreNow()) {
      // Spin the roller to eject the coral once the state machine declares it is ready.
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
      // Clear the coral/algae flags so the subsystem knows it is empty again.
      manipulator.setCoral(false);
      manipulator.setAlgae(false);
    }
    manipulatorSM.atGoalState(false);
  }

  @Override
  public boolean isFinished() {
    // Run the roller for a fixed amount of time to guarantee the coral is released.
    return timer.hasElapsed(1);
    // return manipulator.getMotorPos() > 200;// || !manipulator.hasCoral();
  }
}
