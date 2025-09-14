package frc.robot.commands.DriveCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class CancelledDrive extends Command{
        //imports
    private DriveSubsystem drive;
    /*
     * Command to drive robot with active angling towards reef (usualy has note)
     **/
    public CancelledDrive(DriveSubsystem m_drive) {

        drive = m_drive;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drive);
    }

    // Called when the command is initially scheduled. Not used right now
    @Override
    public void initialize() {
        drive.drive(0, 0, 0, true);
        //nothing
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        //nothing
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
