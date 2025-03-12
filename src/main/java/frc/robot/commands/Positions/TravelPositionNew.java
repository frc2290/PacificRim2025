// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DifferentialArm;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.PositionState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TravelPositionNew extends SequentialCommandGroup {

    private double diffExt1 = 240;
    private double diffRot1 = -30;
    private double elevatorPos1 = 0.375;
    private double diffRot2 = -125;
    private double diffExt2 = 20;

    /** Creates a new Test. */
    public TravelPositionNew(DifferentialSubsystem diffArm, ElevatorSubsystem elevator, StateSubsystem stateSubsystem) {
        Command result;
        if (stateSubsystem.getCurrentState() == PositionState.IntakePosition || stateSubsystem.getCurrentState() == PositionState.StartPosition) {
            Command moveExtStep1 = diffArm.setExtensionSetpointCommand(diffExt1);
            //Command moveRotStep1 = diffArm.setRotationSetpointCommand(diffRot1);
            Command moveElevator = elevator.setElevatorSetpointCommand(Elevator.transportSetpoint);
            ParallelCommandGroup moveAndExtend = new ParallelCommandGroup(moveExtStep1, moveElevator);
            Command moveRotStep2 = diffArm.setRotationSetpointCommand(DifferentialArm.transportRotationSetpoint);
            Command moveExtStep2 = diffArm.setExtensionSetpointCommand(DifferentialArm.transportExtensionSetpoint);
            result = new SequentialCommandGroup(moveAndExtend, moveRotStep2, moveExtStep2);
        } else {
            Command moveRot = diffArm.setRotationSetpointCommand(DifferentialArm.transportRotationSetpoint);
            Command moveExt = diffArm.setExtensionSetpointCommand(DifferentialArm.transportExtensionSetpoint);
            Command moveElev = elevator.setElevatorSetpointCommand(Elevator.transportSetpoint);
            ParallelCommandGroup moveAndRot = new ParallelCommandGroup(moveRot, moveExt);
            result = new SequentialCommandGroup(moveAndRot, moveElev);
        }
        addCommands(result, stateSubsystem.setCurrentStateCommand(PositionState.TravelPosition));
    }
}
