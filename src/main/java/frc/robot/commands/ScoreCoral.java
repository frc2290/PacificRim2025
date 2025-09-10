// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.PositionState;
import frc.utils.PoseEstimatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreCoral extends Command {
    private ManipulatorSubsystem manipulator;
    private DifferentialSubsystem diff;
    private StateSubsystem state;
    private PoseEstimatorSubsystem pose;

    private Timer timer;

    /**
     * Creates a new ScoreCoral using real timers.
     */
    public ScoreCoral(ManipulatorSubsystem m_manip, DifferentialSubsystem m_diff, StateSubsystem m_state, PoseEstimatorSubsystem m_pose) {
        this(m_manip, m_diff, m_state, m_pose, new Timer());
    }

    /**
     * Test-oriented constructor that allows a timer to be supplied for unit testing.
     */
    public ScoreCoral(ManipulatorSubsystem m_manip, DifferentialSubsystem m_diff, StateSubsystem m_state, PoseEstimatorSubsystem m_pose, Timer timer) {
        manipulator = m_manip;
        diff = m_diff;
        state = m_state;
        pose = m_pose;
        this.timer = timer;
        // Use addRequirements() here to declare subsystem dependencies.
        //addRequirements(manipulator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        manipulator.resetMotorPos();
        timer.reset();
        System.out.println("Starting Score Coral");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //if ((pose.atTargetPose(diff.hasLaserCanDistance()) && state.atCurrentState()) || !state.getRotationLock()) {
        if ((pose.atTargetPose(diff.hasLaserCanDistance()) && state.atCurrentState()) || !state.getRotationLock() || state.getCurrentState() == PositionState.L1Position || state.getCurrentState() == PositionState.ProcessorPosition) {
            manipulator.intake(1);
            if (!timer.isRunning()) {
                timer.restart();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        System.out.println("Time to score: " + timer.get());
        manipulator.intake(0);
        if (!interrupted) {
            manipulator.setCoral(false);
            manipulator.setAlgae(false);
            state.setGoal(PositionState.IntakePosition);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
        //return manipulator.getMotorPos() > 200;// || !manipulator.hasCoral();
    }
}
