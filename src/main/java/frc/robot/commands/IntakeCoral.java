// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.DriveState;
import frc.robot.subsystems.StateSubsystem.State;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
    private ManipulatorSubsystem manipulator;
    private StateSubsystem state;

    private Timer currentTimer = new Timer();
    private Timer delayTimer = new Timer();

    /** Creates a new IntakeOn. */
    public IntakeCoral(ManipulatorSubsystem m_manip, StateSubsystem m_state) {
        manipulator = m_manip;
        state = m_state;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentTimer.reset();
        delayTimer.reset();
        state.setGoal(State.IntakePosition);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        manipulator.intake(-0.75);
        System.out.println(manipulator.getOutputCurrent());
        if (manipulator.getOutputCurrent() > 30) {
            if (!delayTimer.isRunning()) {
                delayTimer.restart();
            } else if (delayTimer.hasElapsed(0.5) && delayTimer.isRunning()) {
                delayTimer.stop();
                currentTimer.restart();
            }
        }
        System.out.println("Delay: " + delayTimer.hasElapsed(0.5));
        System.out.println("Current: " + currentTimer.hasElapsed(0.5));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        currentTimer.stop();
        delayTimer.stop();
        manipulator.intake(0);
        manipulator.setCoral(true);
        state.setGoal(State.TravelPosition);
        state.setDriveState(DriveState.Teleop);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return currentTimer.hasElapsed(0.5);
        // return manipulator.gotCoral();
    }
}
