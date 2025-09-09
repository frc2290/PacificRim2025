// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Climber;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ArmStateManager;
import frc.robot.subsystems.ArmStateManager.ElevatorManipulatorState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberOut extends Command {
    private ClimbSubsystem climb;
    private ArmStateManager arm;

    /** Creates a new ClimberOut. */
    public ClimberOut(ClimbSubsystem _climb, ArmStateManager arm) {
        climb = _climb;
        this.arm = arm;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climb);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.setElevManiGoal(ElevatorManipulatorState.Climb);
        climb.setServoOpen();
        climb.setClimbing(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //climb.setServoPos(180);
        //if (climb.getServoPos() > 170) {
        if (arm.atElevManiGoal()) {
            climb.setClimberSetpoint(Climber.climberOutSetpoint);
        }
        //}
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //climb.setClimberSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return climb.climberOut();
    }
}
