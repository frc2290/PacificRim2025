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
public class L1Position extends SequentialCommandGroup {
    private double elevatorPos = 0.125;
    private double diffExt = 80;
    private double diffRot = -225;
    private double diffExt1 = 220;

    /** Creates a new L1PositionNew. */
    public L1Position(DifferentialSubsystem diffArm, ElevatorSubsystem elevator, StateSubsystem stateSubsystem) {
        if (!stateSubsystem.atSafeState()) {
            Command moveExtStep1 = diffArm.setExtensionSetpointCommand(diffExt1);
            // Command moveRotStep1 = diffArm.setRotationSetpointCommand(diffRot1);
            Command moveElevator = elevator.setElevatorSetpointCommand(Elevator.transportSetpoint);
            ParallelCommandGroup moveAndUp = new ParallelCommandGroup(moveExtStep1, moveElevator);
            Command moveExtStep2 = diffArm.setExtensionSetpointCommand(diffExt);
            Command moveRotStep2 = diffArm.setRotationSetpointCommand(diffRot)
                    .andThen(moveExtStep2);
            Command moveElev2 = elevator.setElevatorSetpointCommand(elevatorPos);
            ParallelCommandGroup rotateAndUpRest = new ParallelCommandGroup(moveRotStep2, moveElev2);
            Command moveRot = diffArm.setRotationSetpointCommand(diffRot);
            addCommands(moveAndUp, rotateAndUpRest, moveRot,
                    stateSubsystem.setCurrentStateCommand(PositionState.L1Position));
        } else {
            Command rotTransport = diffArm.setRotationSetpointCommand(DifferentialArm.transportRotationSetpoint);
            Command moveExt = diffArm.setExtensionSetpointCommand(diffExt);
            Command moveElev = elevator.setElevatorSetpointCommand(elevatorPos);
            Command moveRot = diffArm.setRotationSetpointCommand(diffRot);
            ParallelCommandGroup moveExtAndElev = new ParallelCommandGroup(moveElev, moveRot);
            // Add your commands in the addCommands() call, e.g.
            // addCommands(new FooCommand(), new BarCommand());
            addCommands(rotTransport, moveExtAndElev, moveExt,
                    stateSubsystem.setCurrentStateCommand(PositionState.L1Position));
        }
    }
}
