package frc.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Tests for {@link LinearInterpolator}. */
public class LinearInterpolatorTest {
  private LinearInterpolator interpolator;

  @BeforeEach
  void setUp() {
    // Arrange: unsorted so constructor must sort internally
    double[][] data = {
      {10.0, 100.0},
      {1.0, 10.0},
      {3.0, 30.0}
    };
    interpolator = new LinearInterpolator(data);
  }

  @Test
  void reportsInitializedAfterConstruction() {
    // Act & Assert
    assertTrue(interpolator.isInitialized());
  }

  @Test
  void returnsExactValueWhenXMatches() {
    // Act
    double y = interpolator.getInterpolatedValue(3.0);
    // Assert
    assertEquals(30.0, y, 1e-9);
  }

  @Test
  void interpolatesBetweenPoints() {
    // Act
    double y = interpolator.getInterpolatedValue(2.0);
    // Assert
    assertEquals(20.0, y, 1e-9);
  }

  @Test
  void clampsToMinimumWhenBelowRange() {
    // Act
    double y = interpolator.getInterpolatedValue(0.0);
    // Assert
    assertEquals(10.0, y, 1e-9);
  }

  @Test
  void clampsToMaximumWhenAboveRange() {
    // Act
    double y = interpolator.getInterpolatedValue(20.0);
    // Assert
    assertEquals(100.0, y, 1e-9);
  }

  @Test
  void returnsZeroWhenUninitialized() {
    LinearInterpolator empty = new LinearInterpolator(new double[][] {});
    assertFalse(empty.isInitialized());
    assertEquals(0.0, empty.getInterpolatedValue(5.0), 1e-9);
  }
}
