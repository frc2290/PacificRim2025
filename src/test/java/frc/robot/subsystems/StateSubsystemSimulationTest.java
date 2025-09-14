package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.utils.LEDUtility;
import org.junit.jupiter.api.Test;

public class StateSubsystemSimulationTest {
  static class TestStateSubsystem extends StateSubsystem {
    boolean scheduled = false;

    TestStateSubsystem() {
      super(null, null, null, null, null, new LEDUtility(0));
    }

    @Override
    public boolean atCurrentState() {
      return true;
    }

    @Override
    public void periodic() {
      if (getGoalState() != getCurrentState() && !isTransitioning() && DriverStation.isEnabled()) {
        scheduled = true;
      }
    }
  }

  @Test
  public void schedulesOnlyWhenEnabled() {
    assertTrue(HAL.initialize(500, 0));

    TestStateSubsystem state = new TestStateSubsystem();
    state.setCurrentState(StateSubsystem.PositionState.StartPosition);
    state.setGoal(StateSubsystem.PositionState.TravelPosition);

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    state.periodic();
    assertFalse(state.scheduled);

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    state.periodic();
    assertTrue(state.scheduled);
  }
}
