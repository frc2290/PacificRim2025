package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DifferentialSubsystem;

/** Auto routine to test extension and rotation of the differential arm. */
public class DifferentialTestAuto extends SequentialCommandGroup {
  public DifferentialTestAuto(DifferentialSubsystem diff) {
    addCommands(
        Commands.runOnce(() -> System.out.println("Starting Differential Test")),
        Commands.deadline(
            diff.setRotAndExtSetpointCommand(100, 30),
            Commands.run(() -> System.out.println(
                "Extension: " + diff.getExtensionPosition() +
                " Rotation: " + diff.getRotationPosition()))
        ),
        Commands.runOnce(() -> System.out.println("First move complete")),
        Commands.deadline(
            diff.setRotAndExtSetpointCommand(0, 0),
            Commands.run(() -> System.out.println(
                "Extension: " + diff.getExtensionPosition() +
                " Rotation: " + diff.getRotationPosition()))
        ),
        Commands.runOnce(() -> System.out.println("Differential test complete"))
    );
  }
}
