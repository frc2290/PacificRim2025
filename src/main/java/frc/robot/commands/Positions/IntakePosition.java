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
public class IntakePosition extends SequentialCommandGroup {
  /** Creates a new IntakePositionNew. */
  public IntakePosition(DifferentialSubsystem diffArm, ElevatorSubsystem elevator, StateSubsystem stateSubsystem) {
    //Command moveExt1 = diffArm.setExtensionSetpointCommand(70);
    Command moveRotAndExt = diffArm.setRotAndExtSetpointCommand(80, 225);
    Command moveElev = elevator.setElevatorSetpointCommand(Elevator.intakeSetpoint);
    ParallelCommandGroup moveElevAndArm = new ParallelCommandGroup(moveRotAndExt, moveElev);
    Command moveRotAndExt2 = diffArm.setRotAndExtSetpointCommand(DifferentialArm.intakeExtensionSetpoint, DifferentialArm.intakeRotationSetpoint);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(moveElevAndArm, moveRotAndExt2, stateSubsystem.setCurrentStateCommand(PositionState.IntakePosition));
  }
}
