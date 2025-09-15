package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import au.grapplerobotics.LaserCan;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DifferentialArm;
import frc.utils.DifferentialArmSim;
import java.lang.reflect.Field;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

@ExtendWith(MockitoExtension.class)
class DifferentialSubsystemTest {

  @Mock private SparkMax leftMotor;
  @Mock private SparkMax rightMotor;
  @Mock private RelativeEncoder leftEnc;
  @Mock private RelativeEncoder rightEnc;
  @Mock private SparkClosedLoopController leftArm;
  @Mock private SparkClosedLoopController rightArm;
  @Mock private LaserCan laserCan;

  @BeforeAll
  static void setupHal() {
    HAL.initialize(500, 0);
  }

  @BeforeEach
  void setUp() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
    CommandScheduler.getInstance().enable();
  }

  @AfterEach
  void tearDown() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterAllSubsystems();
  }

  @Test
  void extendSetsBothMotors() {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
      diff.extend(0.7);
      verify(leftMotor).set(0.7);
      verify(rightMotor).set(0.7);
    }
  }

  @Test
  void extendPassesThroughOutOfRange() {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
      diff.extend(1.2);
      verify(leftMotor).set(1.2);
      verify(rightMotor).set(1.2);
    }
  }

  @Test
  void rotateSetsOppositeMotorOutputs() {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
      diff.rotate(0.4);
      verify(leftMotor).set(0.4);
      verify(rightMotor).set(-0.4);
    }
  }

  @Test
  void rotateWithNegativeValueInvertsOutputs() {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
      diff.rotate(-0.6);
      verify(leftMotor).set(-0.6);
      verify(rightMotor).set(0.6);
    }
  }

  @Test
  void extensionSetpointAndIncrement() {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
      diff.setExtensionSetpoint(100);
      assertEquals(100, diff.getExtensionSetpoint(), 1e-9);

      Command cmd = diff.incrementExtensionSetpoint(5);
      cmd.initialize();
      assertEquals(105, diff.getExtensionSetpoint(), 1e-9);
    }
  }

  @Test
  void extensionSetpointAcceptsNegative() {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
      diff.setExtensionSetpoint(-10);
      assertEquals(-10, diff.getExtensionSetpoint(), 1e-9);
    }
  }

  @Test
  void incrementExtensionSetpointSubtractsWhenNegative() {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
      diff.setExtensionSetpoint(10);
      diff.incrementExtensionSetpoint(-3).initialize();
      assertEquals(7, diff.getExtensionSetpoint(), 1e-9);
    }
  }

  @Test
  void rotationSetpointAndIncrement() {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
      diff.setRotationSetpoint(10);
      assertEquals(10, diff.getRotationSetpoint(), 1e-9);

      Command cmd = diff.incrementRotationSetpoint(5);
      cmd.initialize();
      assertEquals(15, diff.getRotationSetpoint(), 1e-9);
    }
  }

  @Test
  void incrementRotationSetpointSubtractsWhenNegative() {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
      diff.setRotationSetpoint(10);
      diff.incrementRotationSetpoint(-6).initialize();
      assertEquals(4, diff.getRotationSetpoint(), 1e-9);
    }
  }

  @Test
  void currentDrawSumsMotors() {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
      when(leftMotor.getOutputCurrent()).thenReturn(2.0);
      when(rightMotor.getOutputCurrent()).thenReturn(3.5);
      assertEquals(5.5, diff.getCurrentDraw(), 1e-9);
    }
  }

  @Test
  void encoderPositionsAndDerivedValues() {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
      when(leftEnc.getPosition()).thenReturn(20.0);
      when(rightEnc.getPosition()).thenReturn(40.0);
      assertEquals(20.0, diff.getLeftPos(), 1e-9);
      assertEquals(40.0, diff.getRightPos(), 1e-9);
      assertEquals(30.0, diff.getExtensionPosition(), 1e-9);
    }
  }

  @Test
  void rotationPositionUsesDifference() {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
      when(leftEnc.getPosition()).thenReturn(60.0);
      when(rightEnc.getPosition()).thenReturn(100.0);
      assertEquals(36.0, diff.getRotationPosition(), 1e-9);
    }
  }

  @Test
  void atExtensionSetpointChecksTolerance() {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
      diff.setExtensionSetpoint(100);
      when(leftEnc.getPosition()).thenReturn(104.0);
      when(rightEnc.getPosition()).thenReturn(106.0);
      assertTrue(diff.atExtenstionSetpoint());

      when(leftEnc.getPosition()).thenReturn(120.0);
      when(rightEnc.getPosition()).thenReturn(110.0);
      assertFalse(diff.atExtenstionSetpoint());
    }
  }

  @Test
  void atRotationSetpointChecksTolerance() {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
      diff.setRotationSetpoint(36);
      when(leftEnc.getPosition()).thenReturn(60.0);
      when(rightEnc.getPosition()).thenReturn(100.0);
      assertTrue(diff.atRotationSetpoint());

      when(leftEnc.getPosition()).thenReturn(20.0);
      when(rightEnc.getPosition()).thenReturn(100.0);
      assertFalse(diff.atRotationSetpoint());
    }
  }

  @Test
  void laserCanInterpolatorsUseDistance() throws Exception {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
      Field distance = DifferentialSubsystem.class.getDeclaredField("laserCanDistance");
      distance.setAccessible(true);
      distance.setInt(diff, 200);

      assertEquals(DifferentialArm.l4RotationData[1][1], diff.l4RotationInterpolate(), 1e-9);
      assertEquals(DifferentialArm.l4ExtensionData[1][1], diff.l4ExtensionInterpolate(), 1e-9);
      assertEquals(DifferentialArm.l2_3RotationData[1][1], diff.l2_3RotationInterpolate(), 1e-9);
      assertEquals(DifferentialArm.l2_3ExtensionData[1][1], diff.l2_3ExtensionInterpolate(), 1e-9);
    }
  }

  @Test
  void laserCanAccessorsReflectFields() throws Exception {
    try (DifferentialSubsystem diff =
        new DifferentialSubsystem(
            leftMotor, rightMotor, leftEnc, rightEnc, leftArm, rightArm, laserCan)) {
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

  @Test
  void updateSimStateWritesEncoderValues() throws Exception {
    try (DifferentialSubsystem diff = new DifferentialSubsystem()) {
      DifferentialArmSim mockSim = mock(DifferentialArmSim.class);
      double extMeters = 0.2;
      double rotRads = 0.3;
      double extVel = 1.0;
      double rotVel = 2.0;

      when(mockSim.getExtensionPositionMeters()).thenReturn(extMeters);
      when(mockSim.getRotationAngleRads()).thenReturn(rotRads);
      when(mockSim.getExtensionVelocityMetersPerSec()).thenReturn(extVel);
      when(mockSim.getRotationVelocityRadsPerSec()).thenReturn(rotVel);

      Field armSimField = DifferentialSubsystem.class.getDeclaredField("armSim");
      armSimField.setAccessible(true);
      armSimField.set(diff, mockSim);

      Field leftEncField = DifferentialSubsystem.class.getDeclaredField("leftEncoderSim");
      Field rightEncField = DifferentialSubsystem.class.getDeclaredField("rightEncoderSim");
      leftEncField.setAccessible(true);
      rightEncField.setAccessible(true);
      SparkRelativeEncoderSim leftSim = (SparkRelativeEncoderSim) leftEncField.get(diff);
      SparkRelativeEncoderSim rightSim = (SparkRelativeEncoderSim) rightEncField.get(diff);

      diff.updateSimState(0.02);

      double expectedLeft =
          Units.radiansToRotations(
              extMeters / DifferentialArm.kSimLinearDriveRadiusMeters
                  - rotRads / DifferentialArm.kSimDifferentialArmRadiusMeters);
      double expectedRight =
          Units.radiansToRotations(
              extMeters / DifferentialArm.kSimLinearDriveRadiusMeters
                  + rotRads / DifferentialArm.kSimDifferentialArmRadiusMeters);

      assertEquals(expectedLeft, leftSim.getPosition(), 1e-9);
      assertEquals(expectedRight, rightSim.getPosition(), 1e-9);
    }
  }

  private static class DifferentialArmRig implements AutoCloseable {
    final DifferentialSubsystem diff;
    final SparkMax leftMotor;
    final SparkMax rightMotor;
    final SparkMaxSim leftSim;
    final SparkMaxSim rightSim;

    DifferentialArmRig() throws Exception {
      leftMotor = new SparkMax(100, MotorType.kBrushless);
      rightMotor = new SparkMax(101, MotorType.kBrushless);

      SparkMaxConfig cfg = new SparkMaxConfig();
      leftMotor.configure(
          cfg,
          com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters,
          com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);
      rightMotor.configure(
          cfg,
          com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters,
          com.revrobotics.spark.SparkBase.PersistMode.kNoPersistParameters);

      RelativeEncoder leftEnc = leftMotor.getEncoder();
      RelativeEncoder rightEnc = rightMotor.getEncoder();
      SparkClosedLoopController leftCtrl = leftMotor.getClosedLoopController();
      SparkClosedLoopController rightCtrl = rightMotor.getClosedLoopController();

      diff =
          new DifferentialSubsystem(
              leftMotor, rightMotor, leftEnc, rightEnc, leftCtrl, rightCtrl, null);

      DifferentialArmSim armSim =
          new DifferentialArmSim(
              DifferentialArm.kSimExtensionMassKg,
              DifferentialArm.kSimRotationMassKg,
              DifferentialArm.kSimRotationInertiaKgM2,
              DifferentialArm.kSimComOffsetMeters,
              DifferentialArm.kSimExtensionInclinationRads,
              0.0,
              DifferentialArm.kSimExtensionViscousDamping,
              0.0,
              DifferentialArm.kSimRotationViscousDamping,
              0.0,
              DCMotor.getNEO(1),
              DCMotor.getNEO(1),
              DifferentialArm.kSimLinearDriveRadiusMeters,
              DifferentialArm.kSimDifferentialArmRadiusMeters,
              DifferentialArm.kSimSensorOffsetRads,
              DifferentialArm.kSimMotorRotorInertia,
              DifferentialArm.kSimMinExtensionMeters,
              DifferentialArm.kSimMaxExtensionMeters,
              DifferentialArm.kSimMinThetaRads,
              DifferentialArm.kSimMaxThetaRads,
              DifferentialArm.kSimStartingExtensionMeters,
              DifferentialArm.kSimStartingThetaRads);

      leftSim = new SparkMaxSim(leftMotor, DCMotor.getNEO(1));
      rightSim = new SparkMaxSim(rightMotor, DCMotor.getNEO(1));
      SparkRelativeEncoderSim leftEncSim = leftSim.getRelativeEncoderSim();
      SparkRelativeEncoderSim rightEncSim = rightSim.getRelativeEncoderSim();

      Field armSimField = DifferentialSubsystem.class.getDeclaredField("armSim");
      Field leftSimField = DifferentialSubsystem.class.getDeclaredField("leftSim");
      Field rightSimField = DifferentialSubsystem.class.getDeclaredField("rightSim");
      Field leftEncSimField = DifferentialSubsystem.class.getDeclaredField("leftEncoderSim");
      Field rightEncSimField = DifferentialSubsystem.class.getDeclaredField("rightEncoderSim");
      armSimField.setAccessible(true);
      leftSimField.setAccessible(true);
      rightSimField.setAccessible(true);
      leftEncSimField.setAccessible(true);
      rightEncSimField.setAccessible(true);
      armSimField.set(diff, armSim);
      leftSimField.set(diff, leftSim);
      rightSimField.set(diff, rightSim);
      leftEncSimField.set(diff, leftEncSim);
      rightEncSimField.set(diff, rightEncSim);
    }

    @Override
    public void close() {
      diff.close();
      leftMotor.close();
      rightMotor.close();
    }
  }

  @Test
  void motorsTogetherExtendArm() throws Exception {
    try (DifferentialArmRig rig = new DifferentialArmRig()) {
      for (int i = 0; i < 50; i++) {
        rig.leftSim.setAppliedOutput(1.0);
        rig.rightSim.setAppliedOutput(1.0);
        for (int j = 0; j < 4; j++) {
          rig.diff.updateSimState(0.005);
        }
        SimHooks.stepTiming(0.02);
      }
      assertTrue(rig.diff.getExtensionPosition() > 0.0);
    }
  }

  @Test
  void motorsOppositeRotateArm() throws Exception {
    try (DifferentialArmRig rig = new DifferentialArmRig()) {
      for (int i = 0; i < 50; i++) {
        rig.leftSim.setAppliedOutput(1.0);
        rig.rightSim.setAppliedOutput(-1.0);
        for (int j = 0; j < 4; j++) {
          rig.diff.updateSimState(0.005);
        }
        SimHooks.stepTiming(0.02);
      }
      assertTrue(Math.abs(rig.diff.getRotationPosition()) > 1.0);
    }
  }
}
