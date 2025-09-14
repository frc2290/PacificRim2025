package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DifferentialArm;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class TravelPosition extends SequentialCommandGroup {

    public TravelPosition(DifferentialSubsystem diffArm, ElevatorSubsystem elevator) {
        addCommands(
            // Step 0: Retract + move elevator in parallel
            new ParallelCommandGroup(
                diffArm.setExtensionSetpointCommand(220),
                elevator.setElevatorSetpointCommand(Elevator.transportSetpoint)
            ),

            // Step 1: Rotate arm
            diffArm.setRotationSetpointCommand(DifferentialArm.transportRotationSetpoint),

            // Step 2: Extend to transport length
            diffArm.setExtensionSetpointCommand(DifferentialArm.transportExtensionSetpoint)
        );
    }
}
