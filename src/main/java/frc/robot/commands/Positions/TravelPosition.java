// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.PositionState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TravelPosition extends Command {

    private double elevatorPos = 0.1;
    private double diffExt = 100;
    private double diffRot = 90;

    private double diffExt1 = 240;
    private double diffRot1 = -30;
    private double elevatorPos1 = 0.375;
    private double diffRot2 = -125;
    private double diffExt2 = 20;

    private ElevatorSubsystem elevator;
    private DifferentialSubsystem diff;
    private StateSubsystem state;

    private boolean moved_ext = false;
    private boolean moved_rot = false;
    private boolean moved_elev = false;
    private boolean moved_ext2 = false;
    private boolean moved_rot2 = false;

    /** Creates a new TravelPosition. */
    public TravelPosition(DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator, StateSubsystem m_state) {
        elevator = m_elevator;
        diff = m_diff;
        state = m_state;
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(m_diff, m_elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        moved_ext = false;
        moved_rot = false;
        moved_elev = false;
        moved_ext2 = false;
        moved_rot2 = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (state.getCurrentState() != PositionState.IntakePosition && state.getCurrentState() != PositionState.StartPosition) {
            diff.setRotationSetpoint(diffRot2);
            diff.setExtensionSetpoint(diffExt2);
            if (diff.atExtenstionSetpoint() && moved_ext && diff.atRotationSetpoint() && moved_rot) {
                elevator.setElevatorSetpoint(elevatorPos1);
                if (elevator.atPosition()) {
                    moved_elev = true;
                }
            }
            moved_ext = true;
            moved_rot = true;
            moved_ext2 = true;
            moved_rot2 = true;
        } else {
            // diff.setExtensionSetpoint(diffExt);
            // if (diff.atExtenstionSetpoint() && moved_ext) {
            //     diff.setRotationSetpoint(diffRot);
            //     if (diff.atRotationSetpoint() && moved_rot) {
            //         elevator.setElevatorSetpoint(elevatorPos);
            //         if (elevator.atPosition()) {
            //             moved_elev = true;
            //         }
            //     }
            //     moved_rot = true;
            // }
            // moved_ext = true;
            if (!moved_ext2) diff.setExtensionSetpoint(diffExt1);
            if (diff.atExtenstionSetpoint() && moved_ext2) {
                if (!moved_rot2) diff.setRotationSetpoint(diffRot1);
                if (diff.atRotationSetpoint() && moved_rot2) {
                    elevator.setElevatorSetpoint(elevatorPos1);
                    if (elevator.atPosition() && moved_elev) {
                        if (!moved_rot) diff.setRotationSetpoint(diffRot2);
                        if (diff.atRotationSetpoint() && moved_rot) {
                            diff.setExtensionSetpoint(diffExt2);
                            if (diff.atExtenstionSetpoint()) {
                                moved_ext = true;
                            }
                        }
                        moved_rot = true;
                    }
                    moved_elev = true;
                }
                moved_rot2 = true;
            }
            moved_ext2 = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        state.setCurrentState(PositionState.TravelPosition);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (moved_ext && moved_rot && moved_elev && moved_ext2 && moved_rot2);
        // return false;
    }
}
