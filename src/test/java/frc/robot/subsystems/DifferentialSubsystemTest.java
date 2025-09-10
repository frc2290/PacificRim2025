package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import java.lang.reflect.Field;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import au.grapplerobotics.LaserCan;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.DifferentialArm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@ExtendWith(MockitoExtension.class)
class DifferentialSubsystemTest {

    @Mock private SparkMax leftMotor;
    @Mock private SparkMax rightMotor;
    @Mock private RelativeEncoder leftEnc;
    @Mock private RelativeEncoder rightEnc;
    @Mock private SparkClosedLoopController leftArm;
    @Mock private SparkClosedLoopController rightArm;
    @Mock private LaserCan laserCan;

    private DifferentialSubsystem diff;

    @BeforeEach
    void setUp() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        CommandScheduler.getInstance().enable();

        diff = new DifferentialSubsystem(leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan);
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
    }

    @Test
    void extendSetsBothMotors() {
        diff.extend(0.7);
        verify(leftMotor).set(0.7);
        verify(rightMotor).set(0.7);
    }

    @Test
    void rotateSetsOppositeMotorOutputs() {
        diff.rotate(0.4);
        verify(leftMotor).set(0.4);
        verify(rightMotor).set(-0.4);
    }

    @Test
    void extensionSetpointAndIncrement() {
        diff.setExtensionSetpoint(100);
        assertEquals(100, diff.getExtensionSetpoint(), 1e-9);

        Command cmd = diff.incrementExtensionSetpoint(5);
        cmd.initialize();
        assertEquals(105, diff.getExtensionSetpoint(), 1e-9);
    }

    @Test
    void rotationSetpointAndIncrement() {
        diff.setRotationSetpoint(10);
        assertEquals(10, diff.getRotationSetpoint(), 1e-9);

        Command cmd = diff.incrementRotationSetpoint(5);
        cmd.initialize();
        assertEquals(15, diff.getRotationSetpoint(), 1e-9);
    }

    @Test
    void currentDrawSumsMotors() {
        when(leftMotor.getOutputCurrent()).thenReturn(2.0);
        when(rightMotor.getOutputCurrent()).thenReturn(3.5);
        assertEquals(5.5, diff.getCurrentDraw(), 1e-9);
    }

    @Test
    void encoderPositionsAndDerivedValues() {
        when(leftEnc.getPosition()).thenReturn(20.0);
        when(rightEnc.getPosition()).thenReturn(40.0);
        assertEquals(20.0, diff.getLeftPos(), 1e-9);
        assertEquals(40.0, diff.getRightPos(), 1e-9);
        assertEquals(30.0, diff.getExtensionPosition(), 1e-9);
    }

    @Test
    void rotationPositionUsesDifference() {
        when(leftEnc.getPosition()).thenReturn(60.0);
        when(rightEnc.getPosition()).thenReturn(100.0);
        assertEquals(36.0, diff.getRotationPosition(), 1e-9);
    }

    @Test
    void atExtensionSetpointChecksTolerance() {
        diff.setExtensionSetpoint(100);
        when(leftEnc.getPosition()).thenReturn(104.0);
        when(rightEnc.getPosition()).thenReturn(106.0);
        assertTrue(diff.atExtenstionSetpoint());

        when(leftEnc.getPosition()).thenReturn(120.0);
        when(rightEnc.getPosition()).thenReturn(110.0);
        assertFalse(diff.atExtenstionSetpoint());
    }

    @Test
    void atRotationSetpointChecksTolerance() {
        diff.setRotationSetpoint(36);
        when(leftEnc.getPosition()).thenReturn(60.0);
        when(rightEnc.getPosition()).thenReturn(100.0);
        assertTrue(diff.atRotationSetpoint());

        when(leftEnc.getPosition()).thenReturn(20.0);
        when(rightEnc.getPosition()).thenReturn(100.0);
        assertFalse(diff.atRotationSetpoint());
    }

    @Test
    void laserCanInterpolatorsUseDistance() throws Exception {
        Field distance = DifferentialSubsystem.class.getDeclaredField("laserCanDistance");
        distance.setAccessible(true);
        distance.setInt(diff, 200);

        assertEquals(DifferentialArm.l4RotationData[1][1], diff.l4RotationInterpolate(), 1e-9);
        assertEquals(DifferentialArm.l4ExtensionData[1][1], diff.l4ExtensionInterpolate(), 1e-9);
        assertEquals(DifferentialArm.l2_3RotationData[1][1], diff.l2_3RotationInterpolate(), 1e-9);
        assertEquals(DifferentialArm.l2_3ExtensionData[1][1], diff.l2_3ExtensionInterpolate(), 1e-9);
    }

    @Test
    void laserCanAccessorsReflectFields() throws Exception {
        Field dist = DifferentialSubsystem.class.getDeclaredField("laserCanDistance");
        dist.setAccessible(true);
        dist.setInt(diff, 150);
        Field has = DifferentialSubsystem.class.getDeclaredField("hasLaserCanDistance");
        has.setAccessible(true);
        has.setBoolean(diff, true);

        assertEquals(150, diff.getLaserCanDistance());
        assertTrue(diff.hasLaserCanDistance());
    }
}

