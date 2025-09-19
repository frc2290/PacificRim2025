package frc.robot.commands.DriveCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** Simple command used when the drivetrain should stay still. */
public class CancelledDrive extends Command{
    private DriveSubsystem drive;
    /**
     * Creates a placeholder command that tells the drivetrain to stop and hold position. It keeps
     * the scheduler happy when we need "no command" selected.
     */
    public CancelledDrive(DriveSubsystem m_drive) {

        drive = m_drive;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drive);
    }

    // Called when the command is initially scheduled. Not used right now.
    @Override
    public void initialize() {
        // Stop all motion whenever this command is requested.
        drive.drive(0, 0, 0, true);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        // No action needed; the drivetrain remains idle after the initialize() call.
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
