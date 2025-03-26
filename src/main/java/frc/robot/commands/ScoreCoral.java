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

    private Timer timer = new Timer();

    /** Creates a new ScoreCoral. */
    public ScoreCoral(ManipulatorSubsystem m_manip, DifferentialSubsystem m_diff, StateSubsystem m_state, PoseEstimatorSubsystem m_pose) {
        manipulator = m_manip;
        diff = m_diff;
        state = m_state;
        pose = m_pose;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        manipulator.resetMotorPos();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //if ((pose.atTargetPose(diff.hasLaserCanDistance()) && state.atCurrentState()) || !state.getRotationLock()) {
        if ((pose.atTargetPose() && state.atCurrentState()) || !state.getRotationLock()) {
            if (state.getCurrentState() == PositionState.L1Position) {
                manipulator.intake(0.5);
            } else {
                manipulator.intake(0.75);
            }
            timer.restart();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.stop();
        System.out.println("Time to score: " + timer.get());
        manipulator.intake(0);
        manipulator.setCoral(false);
        state.setGoal(PositionState.IntakePosition);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return manipulator.getMotorPos() > 25;
    }
}
