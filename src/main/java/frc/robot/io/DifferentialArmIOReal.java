package frc.robot.io;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.DifferentialArm;

/** Real hardware implementation of the differential arm IO. */
public class DifferentialArmIOReal implements DifferentialArmIO {
  private final SparkMax leftMotor;
  private final SparkMax rightMotor;
  private final RelativeEncoder leftEnc;
  private final RelativeEncoder rightEnc;
  private final SparkClosedLoopController leftCtrl;
  private final SparkClosedLoopController rightCtrl;
  private final LaserCan laser;
  private int distance;
  private boolean hasDistance;

  public DifferentialArmIOReal() {
    leftMotor = new SparkMax(DifferentialArm.kLeftMotorId, MotorType.kBrushless);
    rightMotor = new SparkMax(DifferentialArm.kRightMotorId, MotorType.kBrushless);

    leftCtrl = leftMotor.getClosedLoopController();
    rightCtrl = rightMotor.getClosedLoopController();
    leftEnc = leftMotor.getEncoder();
    rightEnc = rightMotor.getEncoder();
    leftEnc.setPosition(0.0);
    rightEnc.setPosition(0.0);

    SparkMaxConfig cfg = new SparkMaxConfig();
    cfg.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(50);
    cfg.encoder
        .positionConversionFactor(DifferentialArm.kEncoderPositionFactor)
        .velocityConversionFactor(DifferentialArm.kEncoderVelocityFactor);
    cfg.closedLoop
        .p(DifferentialArm.v_kp, com.revrobotics.spark.ClosedLoopSlot.kSlot0)
        .i(DifferentialArm.v_ki, com.revrobotics.spark.ClosedLoopSlot.kSlot0)
        .d(DifferentialArm.v_kd, com.revrobotics.spark.ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1);
    leftMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig rightCfg = new SparkMaxConfig();
    rightCfg.apply(cfg).inverted(false);
    rightMotor.configure(rightCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    laser = new LaserCan(DifferentialArm.kLaserCanId);
    try {
      laser.setRangingMode(LaserCan.RangingMode.SHORT);
      laser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      laser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("LaserCan Configuration failed! " + e);
    }
  }

  @Override
  public void setArmVelocitySetpoints(double left, double right) {
    leftCtrl.setReference(left, ControlType.kVelocity);
    rightCtrl.setReference(right, ControlType.kVelocity);
  }

  @Override
  public double getLeftPosition() {
    return leftEnc.getPosition();
  }

  @Override
  public double getRightPosition() {
    return rightEnc.getPosition();
  }

  @Override
  public double getLeftVelocity() {
    return leftEnc.getVelocity();
  }

  @Override
  public double getRightVelocity() {
    return rightEnc.getVelocity();
  }

  @Override
  public double getLeftCurrentAmps() {
    return leftMotor.getOutputCurrent();
  }

  @Override
  public double getRightCurrentAmps() {
    return rightMotor.getOutputCurrent();
  }

  @Override
  public int getLaserDistanceMm() {
    return distance;
  }

  @Override
  public boolean hasLaserDistance() {
    return hasDistance;
  }

  @Override
  public void update() {
    LaserCan.Measurement m = laser.getMeasurement();
    if (m != null
        && m.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        && m.distance_mm < 430) {
      distance = m.distance_mm;
      hasDistance = true;
    } else {
      hasDistance = false;
    }
  }
}
