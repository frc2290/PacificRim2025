// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.StateSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Test extends SequentialCommandGroup {

    private double diffExt1 = 240;
    private double diffRot1 = -30;
    private double elevatorPos1 = 0.375;
    private double diffRot2 = -125;
    private double diffExt2 = 20;

    /** Creates a new Test. */
    public Test(ElevatorSubsystem elevator, DifferentialSubsystem diffArm, StateSubsystem stateSubsystem) {
        Command moveExtStep1 = diffArm.setExtensionSetpointCommand(diffExt1);
        Command moveRotStep1 = diffArm.setRotationSetpointCommand(diffRot1);
        Command moveElevator = elevator.setElevatorSetpointCommand(elevatorPos1);
        Command moveRotStep2 = diffArm.setRotationSetpointCommand(diffRot2);
        Command moveExtStep2 = diffArm.setExtensionSetpointCommand(diffExt2);
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(moveExtStep1, moveRotStep1, moveElevator, moveRotStep2, moveExtStep2);
    }
}

// if (!moved_ext2) diff.setExtensionSetpoint(diffExt1);
// if (diff.atExtenstionSetpoint() && moved_ext2) {
//     if (!moved_rot2) diff.setRotationSetpoint(diffRot1);
//     if (diff.atRotationSetpoint() && moved_rot2) {
//         elevator.setElevatorSetpoint(elevatorPos1);
//         if (elevator.atPosition() && moved_elev) {
//             if (!moved_rot) diff.setRotationSetpoint(diffRot2);
//             if (diff.atRotationSetpoint() && moved_rot) {
//                 diff.setExtensionSetpoint(diffExt2);
//                 if (diff.atExtenstionSetpoint()) {
//                     moved_ext = true;
//                 }
//             }
//             moved_rot = true;
//         }
//         moved_elev = true;
//     }
//     moved_rot2 = true;
// }
// moved_ext2 = true;
