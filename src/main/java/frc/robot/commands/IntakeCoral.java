// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
    private ManipulatorSubsystem manipulator;

    private Timer currentTimer = new Timer();
    private Timer delayTimer = new Timer();

    private boolean sawCoral = false;
    private boolean finished = false;

    /** Creates a new IntakeOn. */
    public IntakeCoral(ManipulatorSubsystem m_manip) {
        manipulator = m_manip;
        addRequirements(manipulator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentTimer.reset();
        delayTimer.reset();
        sawCoral = false;
        finished = false;
        // state manager goal handled elsewhere
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        manipulator.intake(0.55);
        if (!sawCoral && manipulator.seesCoral()) {
            sawCoral = true;
        }
        if (sawCoral) {
            if (!manipulator.seesCoral()) {
                finished = true;
            }
        }
        /*manipulator.intake(-0.9);
        if (manipulator.getOutputCurrent() > 30) {
            if (!delayTimer.isRunning()) {
                delayTimer.restart();
            } else if (delayTimer.hasElapsed(0.5) && delayTimer.isRunning()) {
                delayTimer.stop();
                currentTimer.restart();
            }
        }*/
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        currentTimer.stop();
        delayTimer.stop();
        if (!interrupted) {
            manipulator.intake(0);
            manipulator.setCoral(true);
        }
        // drive state manager updates handled externally
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return finished;
        //return currentTimer.hasElapsed(0.5);
        // return manipulator.gotCoral();
    }
}
