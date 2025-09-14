package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;

/** Simple auto routine to exercise the elevator in simulation. */
public class ElevatorTestAuto extends SequentialCommandGroup {
  public ElevatorTestAuto(ElevatorSubsystem elevator) {
    addCommands(
        Commands.runOnce(
            () -> {
              System.out.println("Starting Elevator Test");
            }),
        Commands.deadline(
            elevator.setElevatorSetpointCommand(0.25),
            Commands.run(
                () ->
                    System.out.println(
                        "Elevator pos: "
                            + elevator.getPosition()
                            + " setpoint: "
                            + elevator.getElevatorSetpoint()))),
        Commands.runOnce(() -> System.out.println("Elevator reached first setpoint")),
        Commands.deadline(
            elevator.setElevatorSetpointCommand(0.0),
            Commands.run(
                () ->
                    System.out.println(
                        "Elevator pos: "
                            + elevator.getPosition()
                            + " setpoint: "
                            + elevator.getElevatorSetpoint()))),
        Commands.runOnce(
            () -> {
              System.out.println("Elevator test complete");
            }));
  }
}
