package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double turningFactor = 2 * Math.PI;
      // SPARK velocity feedforward expects a value in percent output per RPM.
      // Use the motor's free speed in RPM so that a setpoint equal to the
      // free speed results in full output from the controller.
      double drivingVelocityFeedForward = 1.0 / Constants.NeoMotorConstants.kFreeSpeedRpm;
      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(ModuleConstants.kDriveCurrentLimitAmps);
      drivingConfig
          .encoder
          .positionConversionFactor(ModuleConstants.kDriveEncoderPositionFactor) // meters
          .velocityConversionFactor(
              ModuleConstants.kDriveEncoderVelocityFactor); // meters per second
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.012343, 0, 0) // .pid(0.05, 0, 0) //.pid(0.012343, 0, 0) //.pid(0.05, 0, 0)
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);
      drivingConfig.voltageCompensation(12);

      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
      turningConfig
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // radians per second
      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(
              4.3865, 0,
              0) // .pid(1, 0, 0)//.pid((4.3865/2.0), 0, 0) //.pid(4.3865, 0, 0) //.pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
      turningConfig.voltageCompensation(12);
    }
  }
}
