package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.FieldConstants;

public class ManipulatorSimulationTest {
    @Test
    public void limitSwitchSequenceToggles() {
        assertTrue(HAL.initialize(500, 0));

        DriveSubsystem drive = new DriveSubsystem();
        ManipulatorSubsystem manip = new ManipulatorSubsystem(drive);

        assertTrue(manip.hasCoral());
        manip.setCoral(false);

        drive.getField().setRobotPose(FieldConstants.CoralStation.leftCenterFace);
        manip.simulationPeriodic();

        Timer.delay(0.06);
        assertTrue(manip.seesCoral());

        Timer.delay(0.06);
        manip.simulationPeriodic();

        Timer.delay(0.06);
        assertFalse(manip.seesCoral());
        assertTrue(manip.hasCoral());
    }
}
