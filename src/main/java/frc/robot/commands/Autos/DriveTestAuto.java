package frc.robot.commands.Autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PoseEstimatorSubsystem;

/** Auto to drive forward while printing chassis speeds and pose. */
public class DriveTestAuto extends SequentialCommandGroup {
  public DriveTestAuto(DriveSubsystem drive, PoseEstimatorSubsystem pose) {
    addRequirements(drive);
    final double[] lastPrint = {0.0};
    addCommands(
        Commands.runOnce(() -> System.out.println("Starting Drive Test")),
        Commands.run(() -> {
          drive.drive(1.0, 0.0, 0.0, false);
          double now = Timer.getFPGATimestamp();
          if (now - lastPrint[0] > 0.2) {
            lastPrint[0] = now;
            ChassisSpeeds speeds = pose.getChassisSpeeds();
            double[] currents = drive.getModuleCurrents();
            double[] outputs = drive.getDriveAppliedOutputs();
            System.out.printf(
                "vx: %.2f vy: %.2f omega: %.2f pose: %s curr: [%.1f %.1f %.1f %.1f] out: [%.2f %.2f %.2f %.2f]%n",
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                pose.getCurrentPose(),
                currents[0],
                currents[1],
                currents[2],
                currents[3],
                outputs[0],
                outputs[1],
                outputs[2],
                outputs[3]);
          }
        }).withTimeout(1.0),
        Commands.runOnce(() -> {
          drive.drive(0.0, 0.0, 0.0, false);
          System.out.println("Drive test complete, final pose: " + pose.getCurrentPose());
        })
    );
  }
}
