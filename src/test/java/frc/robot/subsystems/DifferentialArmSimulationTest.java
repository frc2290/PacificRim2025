package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Constants.DifferentialArm;
import frc.utils.DifferentialArmSim;
import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;

/** Tests for the differential arm simulation model. */
public class DifferentialArmSimulationTest {

  private static class DifferentialArmRig implements AutoCloseable {
    final DifferentialSubsystem diff;
    final SparkMax leftMotor;
    final SparkMax rightMotor;
    final SparkMaxSim leftSim;
    final SparkMaxSim rightSim;

    DifferentialArmRig() throws Exception {
      leftMotor = new SparkMax(100, MotorType.kBrushless);
      rightMotor = new SparkMax(101, MotorType.kBrushless);

      // minimal configuration to enable encoder sims
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
  public void motorsTogetherExtendArm() throws Exception {
    assertTrue(HAL.initialize(500, 0));
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
  public void motorsOppositeRotateArm() throws Exception {
    assertTrue(HAL.initialize(500, 0));
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
