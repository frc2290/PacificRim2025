package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ManipulatorSubsystem;

/** Auto routine to spin the manipulator intake for verification. */
public class ManipulatorTestAuto extends SequentialCommandGroup {
  public ManipulatorTestAuto(ManipulatorSubsystem manip) {
    addCommands(
        Commands.runOnce(() -> System.out.println("Starting Manipulator Test")),
        Commands.run(() -> {
          manip.intake(0.5);
          System.out.println(
              "Pos: " + manip.getMotorPos() +
              " Current: " + manip.getCurrentDraw());
        }).withTimeout(2.0),
        Commands.runOnce(() -> {
          manip.intake(-0.5);
          System.out.println("Reversing manipulator");
        }),
        Commands.run(() -> {
          System.out.println(
              "Pos: " + manip.getMotorPos() +
              " Current: " + manip.getCurrentDraw());
        }).withTimeout(2.0),
        Commands.runOnce(() -> {
          manip.intake(0.0);
          System.out.println("Manipulator test complete");
        })
    );
  }
}
