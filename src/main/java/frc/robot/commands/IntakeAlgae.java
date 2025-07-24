// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.DriveState;
import frc.robot.subsystems.StateSubsystem.PositionState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgae extends Command {
    private ManipulatorSubsystem manipulator;
    private StateSubsystem state;

    private Timer currentTimer = new Timer();
    private Timer delayTimer = new Timer();

    /** Creates a new IntakeOn. */
    public IntakeAlgae(ManipulatorSubsystem m_manip, StateSubsystem m_state) {
        manipulator = m_manip;
        state = m_state;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(manipulator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentTimer.reset();
        delayTimer.reset();
        //state.setGoal(PositionState.IntakePosition);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        manipulator.intake(-0.9);
        if (manipulator.getOutputCurrent() > 30) {
            if (!delayTimer.isRunning()) {
                delayTimer.restart();
            } else if (delayTimer.hasElapsed(1) && delayTimer.isRunning()) {
                delayTimer.stop();
                currentTimer.restart();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        currentTimer.stop();
        delayTimer.stop();
        if (!interrupted) {
            manipulator.intake(-0.5);
            manipulator.setAlgae(true);
            state.setGoal(PositionState.ProcessorPosition);
        }
        // if (!state.isAuto()) {
        //     state.setDriveState(DriveState.Teleop);
        //     //state.setGoal(PositionState.TravelPosition);
        // }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //return finished;
        return currentTimer.hasElapsed(0.5);
        // return manipulator.gotCoral();
    }
}
