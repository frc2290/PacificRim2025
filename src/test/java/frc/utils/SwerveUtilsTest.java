package frc.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/** Tests for the {@link SwerveUtils} helper class. */
public class SwerveUtilsTest {

  @Test
  void stepTowardsIncreasesWhenBelowTarget() {
    // Arrange
    double current = 0.0;
    double target = 10.0;
    double step = 3.0;
    // Act
    double result = SwerveUtils.StepTowards(current, target, step);
    // Assert
    assertEquals(3.0, result, 1e-9);
  }

  @Test
  void stepTowardsDecreasesWhenAboveTarget() {
    // Arrange
    double current = 10.0;
    double target = 0.0;
    double step = 3.0;
    // Act
    double result = SwerveUtils.StepTowards(current, target, step);
    // Assert
    assertEquals(7.0, result, 1e-9);
  }

  @Test
  void stepTowardsReturnsTargetWhenWithinStep() {
    // Arrange
    double current = 1.0;
    double target = 2.0;
    double step = 5.0;
    // Act
    double result = SwerveUtils.StepTowards(current, target, step);
    // Assert
    assertEquals(target, result, 1e-9);
  }

  @Test
  void stepTowardsCircularMovesPositiveDirection() {
    // Arrange
    double current = 0.0;
    double target = Math.PI / 2.0;
    double step = 0.1;
    // Act
    double result = SwerveUtils.StepTowardsCircular(current, target, step);
    // Assert
    assertEquals(0.1, result, 1e-9);
  }

  @Test
  void stepTowardsCircularWrapsAndMovesNegativeDirection() {
    // Arrange
    double current = 0.0;
    double target = 3.0 * Math.PI / 2.0; // 270 degrees, should wrap negatively
    double step = 0.1;
    // Act
    double result = SwerveUtils.StepTowardsCircular(current, target, step);
    // Assert
    assertEquals(2.0 * Math.PI - 0.1, result, 1e-9);
  }

  @Test
  void stepTowardsCircularReturnsTargetWhenReachableWithWrap() {
    // Arrange
    double current = 0.0;
    double target = 2.0 * Math.PI - 0.5;
    double step = 1.0;
    // Act
    double result = SwerveUtils.StepTowardsCircular(current, target, step);
    // Assert
    assertEquals(target, result, 1e-9);
  }

  @Test
  void angleDifferenceAcrossZeroIsShortestPath() {
    // Arrange
    double a = 0.0;
    double b = 3.0 * Math.PI / 2.0;
    // Act
    double diff = SwerveUtils.AngleDifference(a, b);
    // Assert
    assertEquals(Math.PI / 2.0, diff, 1e-9);
  }

  @Test
  void wrapAngleNormalizesNegativeAngle() {
    // Arrange
    double angle = -Math.PI / 2.0;
    // Act
    double wrapped = SwerveUtils.WrapAngle(angle);
    // Assert
    assertEquals(3.0 * Math.PI / 2.0, wrapped, 1e-9);
  }

  @Test
  void wrapAngleNormalizesLargePositiveAngle() {
    // Arrange
    double angle = 5.0 * Math.PI;
    // Act
    double wrapped = SwerveUtils.WrapAngle(angle);
    // Assert
    assertEquals(Math.PI, wrapped, 1e-9);
  }

  @Test
  void stepTowardsHandlesNegativeStepSize() {
    double result = SwerveUtils.StepTowards(0.0, 1.0, -0.25);
    assertEquals(0.25, result, 1e-9);
  }

  @Test
  void stepTowardsCircularHandlesNegativeStepSize() {
    double result = SwerveUtils.StepTowardsCircular(0.0, 1.0, -0.5);
    assertEquals(0.5, result, 1e-9);
  }
}
