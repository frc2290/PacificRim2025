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
public class TravelPosition extends SequentialCommandGroup {

  private double diffExt1 = 220;

  /** Creates a new Test. */
  public TravelPosition(
      DifferentialSubsystem diffArm, ElevatorSubsystem elevator, StateSubsystem stateSubsystem) {
    Command result;
    if (stateSubsystem.atSafeState()) {
      Command moveExtStep1 = diffArm.setExtensionSetpointCommand(diffExt1);
      // Command moveRotStep1 = diffArm.setRotationSetpointCommand(diffRot1);
      Command moveElevator = elevator.setElevatorSetpointCommand(Elevator.transportSetpoint);
      Command moveRotStep2 =
          diffArm.setRotationSetpointCommand(DifferentialArm.transportRotationSetpoint);
      ParallelCommandGroup moveAndUp = new ParallelCommandGroup(moveExtStep1, moveElevator);
      Command moveExtStep2 =
          diffArm.setExtensionSetpointCommand(DifferentialArm.transportExtensionSetpoint);
      result = new SequentialCommandGroup(moveAndUp, moveRotStep2, moveExtStep2);
    } else {
      Command moveAndRot =
          diffArm.setRotAndExtSetpointCommand(
              DifferentialArm.transportExtensionSetpoint,
              DifferentialArm.transportRotationSetpoint);
      Command moveElev = elevator.setElevatorSetpointCommand(Elevator.transportSetpoint);
      result = new SequentialCommandGroup(moveAndRot, moveElev);
    }
    addCommands(result, stateSubsystem.setCurrentStateCommand(PositionState.TravelPosition));
  }
}
