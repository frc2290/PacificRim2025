package frc.robot.commands.Autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PoseEstimatorSubsystem;

/** Auto to drive forward while printing chassis speeds and pose. */
public class DriveTestAuto extends SequentialCommandGroup {
  public DriveTestAuto(DriveSubsystem drive, PoseEstimatorSubsystem pose) {
    addRequirements(drive);
    addCommands(
        Commands.runOnce(() -> System.out.println("Starting Drive Test")),
        Commands.run(() -> {
          drive.drive(1.0, 0.0, 0.0, false);
          ChassisSpeeds speeds = pose.getChassisSpeeds();
          System.out.printf(
              "vx: %.2f vy: %.2f omega: %.2f pose: %s%n",
              speeds.vxMetersPerSecond,
              speeds.vyMetersPerSecond,
              speeds.omegaRadiansPerSecond,
              pose.getCurrentPose());
        }).withTimeout(2.0),
        Commands.runOnce(() -> {
          drive.drive(0.0, 0.0, 0.0, false);
          System.out.println("Drive test complete, final pose: " + pose.getCurrentPose());
        })
    );
  }
}
