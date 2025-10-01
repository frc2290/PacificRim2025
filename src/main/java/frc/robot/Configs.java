// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ModuleConstants;

/** Utility container for configuration objects that are shared across the robot. */
public final class Configs {
  /**
   * Configuration package for every MAXSwerve module on the drivetrain. The static block ensures
   * that each module receives identical settings every time the code starts.
   */
  public static final class MAXSwerveModule {
    /** Spark Flex configuration that is applied to every driving motor controller. */
    public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();

    /** Spark MAX configuration that is applied to every turning motor controller. */
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor =
          ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = ModuleConstants.kTurningEncoderFactor;
      double drivingVelocityFeedForward = ModuleConstants.kDrivingVelocityFF;
      // double turnPositionFeedforward = 0.31697;

      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(ModuleConstants.kDrivingCurrentLimit);
      drivingConfig
          .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to tune for your own robot!
          .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);
      drivingConfig.voltageCompensation(ModuleConstants.kNominalVoltage);

      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(ModuleConstants.kTurningCurrentLimit);
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
          // These are example gains you may need to tune for your own robot!
          .pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
      turningConfig.voltageCompensation(ModuleConstants.kNominalVoltage);
    }
  }
}
