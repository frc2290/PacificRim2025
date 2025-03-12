// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DifferentialArm;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.StateSubsystem;
import frc.robot.subsystems.StateSubsystem.PositionState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L1PositionNew extends SequentialCommandGroup {
  private double elevatorPos = 0.375;
  private double diffExt = 80;
  private double diffRot = -105;

  /** Creates a new L1PositionNew. */
  public L1PositionNew(DifferentialSubsystem diffArm, ElevatorSubsystem elevator, StateSubsystem stateSubsystem) {
    Command rotTransport = diffArm.setRotationSetpointCommand(DifferentialArm.transportRotationSetpoint);
    Command moveExt = diffArm.setExtensionSetpointCommand(diffExt);
    Command moveElev = elevator.setElevatorSetpointCommand(elevatorPos);
    ParallelCommandGroup moveExtAndElev = new ParallelCommandGroup(moveElev, moveExt);
    Command moveRot = diffArm.setRotationSetpointCommand(diffRot);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(rotTransport, moveExtAndElev, moveRot, stateSubsystem.setCurrentStateCommand(PositionState.L1Position));
  }
}
