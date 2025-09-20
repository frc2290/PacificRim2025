package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManipulatorStateMachine;
import frc.robot.subsystems.ManipulatorSubsystem;

public class IntakeAlgae extends Command{
    private ManipulatorSubsystem manipulator;

    private Timer currentTimer = new Timer();
    private Timer delayTimer = new Timer();

    /** Creates a new IntakeOn. */
    public IntakeAlgae(ManipulatorSubsystem m_manip) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(manipulator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentTimer.reset();
        delayTimer.reset();
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
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return currentTimer.hasElapsed(0.5);
    }
}
