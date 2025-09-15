package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.io.ManipulatorIO;

/** Simple manipulator subsystem wrapping IO implementations. */
public class ManipulatorSubsystem extends SubsystemBase {
  private final ManipulatorIO io;

  public ManipulatorSubsystem(ManipulatorIO io) {
    this.io = io;
  }

  /** Run the intake motor with the given fraction of bus voltage. */
  public void intake(double power) {
    io.setVoltage(power);
  }

  public Command runIntake(double power) {
    return Commands.runOnce(() -> intake(power));
  }

  public double getWristPos() {
    return io.getWristPosition();
  }

  public double getOutputCurrent() {
    return io.getCurrentAmps();
  }

  /** Legacy accessor for commands expecting getCurrentDraw. */
  public double getCurrentDraw() {
    return getOutputCurrent();
  }

  public double getMotorPos() {
    return io.getPositionRotations();
  }

  public double getRPM() {
    return io.getVelocityRPM();
  }

  public void resetMotorPos() {
    io.resetPosition();
  }

  public boolean hasCoral() {
    return io.hasCoral();
  }

  public void setCoral(boolean coral) {
    io.setCoral(coral);
  }

  public boolean hasAlgae() {
    return io.hasAlgae();
  }

  public void setAlgae(boolean algae) {
    io.setAlgae(algae);
  }

  public boolean seesCoral() {
    return io.seesCoral();
  }

  public Trigger hasCoralTrigger() {
    return new Trigger(this::hasCoral);
  }

  public Trigger hasAlgaeTrigger() {
    return new Trigger(this::hasAlgae);
  }

  @Override
  public void periodic() {
    io.update();
  }
}
