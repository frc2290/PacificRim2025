package frc.robot.commands.Autos;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        Commands.run(
                () -> {
                  // Drive at a reduced speed to keep the test manageable
                  drive.drive(0.1, 0.0, 0.0, false);
                  double now = Timer.getFPGATimestamp();
                  if (now - lastPrint[0] > 0.2) {
                    lastPrint[0] = now;
                    ChassisSpeeds speeds = pose.getChassisSpeeds();
                    double[] currents = drive.getModuleCurrents();
                    double[] outputs = drive.getDriveAppliedOutputs();
                    double battery = RobotController.getBatteryVoltage();
                    if (currents.length == 4 && outputs.length == 4) {
                      System.out.printf(
                          "vx: %.2f vy: %.2f omega: %.2f pose: %s curr: [%.1f %.1f %.1f %.1f] out: [%.2f %.2f %.2f %.2f] bat: %.2f%n",
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
                          outputs[3],
                          battery);
                      SmartDashboard.putNumberArray("Drive/ModuleCurrents", currents);
                      SmartDashboard.putNumberArray("Drive/AppliedOutputs", outputs);
                    } else {
                      System.out.printf(
                          "vx: %.2f vy: %.2f omega: %.2f pose: %s bat: %.2f%n",
                          speeds.vxMetersPerSecond,
                          speeds.vyMetersPerSecond,
                          speeds.omegaRadiansPerSecond,
                          pose.getCurrentPose(),
                          battery);
                    }
                    SmartDashboard.putNumber("Drive/BatteryVoltage", battery);
                  }
                })
            .withTimeout(1.0),
        // Once the drive test ends, reset drive state so other commands can take over
        Commands.runOnce(
            () -> {
              // Stop the drivetrain and ensure the rotation PID is centered on the current heading
              drive.drive(0.0, 0.0, 0.0, false);
              double currentHeading = pose.getCurrentPose().getRotation().getDegrees();
              drive.getRotPidController().setSetpoint(currentHeading);
              drive.getRotPidController().reset();

              System.out.println("Drive test complete, final pose: " + pose.getCurrentPose());
            }));
  }
}
