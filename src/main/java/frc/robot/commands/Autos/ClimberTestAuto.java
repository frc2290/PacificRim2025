package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSubsystem;

/** Auto routine to exercise the climber mechanism. */
public class ClimberTestAuto extends SequentialCommandGroup {
  public ClimberTestAuto(ClimbSubsystem climb) {
    addCommands(
        Commands.runOnce(
            () -> {
              System.out.println("Starting Climber Test");
              climb.setServoOpen();
            }),
        Commands.run(
                () -> {
                  climb.setClimberSpeed(0.5);
                  System.out.println("Climb pos: " + climb.getClimberPos());
                })
            .withTimeout(2.0),
        Commands.runOnce(
            () -> {
              climb.setClimberSpeed(-0.5);
              System.out.println("Reversing climber");
            }),
        Commands.run(
                () -> {
                  System.out.println("Climb pos: " + climb.getClimberPos());
                })
            .withTimeout(2.0),
        Commands.runOnce(
            () -> {
              climb.setClimberSpeed(0.0);
              climb.setServoClose();
              System.out.println("Climber test complete");
            }));
  }
}
