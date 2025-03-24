// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberIn extends Command {
    private ClimbSubsystem climb;
    private DriveSubsystem drive;

    /** Creates a new ClimberIn. */
    public ClimberIn(ClimbSubsystem _climb, DriveSubsystem _drive) {
        climb = _climb;
        drive = _drive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climb, drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //climb.setServoClose();
        //drive.setDriveCoast();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drive.drive(-(climb.climbDistance / climb.tClimb), 0, 0, false);
        climb.setClimberSetpoint(Climber.climberInSetpoint);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        climb.setServoClose();
        climb.stopClimberMotor();
        climb.setClimbing(true);
        drive.drive(0, 0, 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return climb.atClimberSetpoint();
        //return true;
    }
}
