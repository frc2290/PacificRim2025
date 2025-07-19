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
public class AlgaeL3Position extends SequentialCommandGroup {
  private double elevatorPos = 1.025;
  private double diffExt = 185;
  private double diffRot = 225;
  private double diffExt1 = 220;

  /** Creates a new AlgaeL3Position. */
  public AlgaeL3Position(DifferentialSubsystem diffArm, ElevatorSubsystem elevator, StateSubsystem stateSubsystem) {
    //if (stateSubsystem.atSafeState()) {
      //Command moveExtStep1 = diffArm.setExtensionSetpointCommand(diffExt1);
      // Command moveRotStep1 = diffArm.setRotationSetpointCommand(diffRot1);
      //Command moveElevator = elevator.setElevatorSetpointCommand(Elevator.transportSetpoint);
      //ParallelCommandGroup moveAndUp = new ParallelCommandGroup(moveExtStep1, moveElevator);
      Command moveExtStep2 = diffArm.setExtensionSetpointCommand(diffExt);
      Command moveRotStep2 = diffArm.setRotationSetpointCommand(diffRot);
      Command moveElev2 = elevator.setElevatorSetpointCommand(elevatorPos);
      //ParallelCommandGroup rotateAndUpRest = new ParallelCommandGroup(moveRotStep2, moveElev2);
      addCommands(moveExtStep2, moveRotStep2, moveElev2, stateSubsystem.setCurrentStateCommand(PositionState.AlgaeL3Position));
    // } else {
    //   Command rotTransport = diffArm.setRotationSetpointCommand(DifferentialArm.transportRotationSetpoint);
    //   Command moveExt = diffArm.setExtensionSetpointCommand(diffExt);
    //   Command moveElev = elevator.setElevatorSetpointCommand(elevatorPos);
    //   ParallelCommandGroup moveExtAndElev = new ParallelCommandGroup(moveElev, moveExt);
    //   Command moveRot = diffArm.setRotationSetpointCommand(diffRot);
    //   // Add your commands in the addCommands() call, e.g.
    //   // addCommands(new FooCommand(), new BarCommand());
    //   addCommands(rotTransport, moveExtAndElev, moveRot,
    //       stateSubsystem.setCurrentStateCommand(PositionState.AlgaeL3Position));
    // }
  }
}
