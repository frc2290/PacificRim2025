package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.io.ElevatorIO;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ElevatorSubsystemTest {
  private static final double EPSILON = 1e-9;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void resetNetworkTables() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.stopLocal();
    inst.startLocal();
  }

  @Test
  void periodicPublishesTelemetryUpdates() {
    LoggingElevatorIO io = new LoggingElevatorIO();
    ElevatorSubsystem subsystem = new ElevatorSubsystem(io);

    subsystem.setElevatorSetpoint(0.75);
    io.positionMeters = 0.5;
    io.velocityMetersPerSecond = -0.2;
    io.currentDrawAmps = 10.0;

    try (var positionSub =
            NetworkTableInstance.getDefault()
                .getDoubleTopic("/Elevator/Position (m)")
                .subscribe(Double.NaN);
        var setpointSub =
            NetworkTableInstance.getDefault()
                .getDoubleTopic("/Elevator/Setpoint (m)")
                .subscribe(Double.NaN);
        var velocitySub =
            NetworkTableInstance.getDefault()
                .getDoubleTopic("/Elevator/Velocity (mps)")
                .subscribe(Double.NaN);
        var currentSub =
            NetworkTableInstance.getDefault()
                .getDoubleTopic("/Elevator/Current Draw (A)")
                .subscribe(Double.NaN)) {
      subsystem.periodic();
      flushNetworkTables();

      assertEquals(0.5, positionSub.get(), EPSILON);
      assertEquals(0.75, setpointSub.get(), EPSILON);
      assertEquals(-0.2, velocitySub.get(), EPSILON);
      assertEquals(10.0, currentSub.get(), EPSILON);

      subsystem.setElevatorSetpoint(1.25);
      io.positionMeters = 1.1;
      io.velocityMetersPerSecond = 0.5;
      io.currentDrawAmps = 8.0;

      subsystem.periodic();
      flushNetworkTables();

      assertEquals(1.1, positionSub.get(), EPSILON);
      assertEquals(1.25, setpointSub.get(), EPSILON);
      assertEquals(0.5, velocitySub.get(), EPSILON);
      assertEquals(8.0, currentSub.get(), EPSILON);
    }
  }

  private static void flushNetworkTables() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    inst.flushLocal();
    inst.flush();
  }

  private static final class LoggingElevatorIO implements ElevatorIO {
    double positionMeters;
    double velocityMetersPerSecond;
    double currentDrawAmps;

    @Override
    public double getPositionMeters() {
      return positionMeters;
    }

    @Override
    public double getVelocityMetersPerSec() {
      return velocityMetersPerSecond;
    }

    @Override
    public double getCurrentDrawAmps() {
      return currentDrawAmps;
    }
  }
}
