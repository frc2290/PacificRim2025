package frc.utils.FLYTLib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class FunctionsTest {

    @Test
    void deadZoneAppliesThreshold() {
        assertEquals(0.0, Functions.DeadZone(0.05, 0.1), 1e-9);
        assertEquals(0.2, Functions.DeadZone(0.2, 0.1), 1e-9);
    }

    @Test
    void exponentialPreservesSign() {
        assertEquals(4.0, Functions.Exponential(2.0), 1e-9);
        assertEquals(-4.0, Functions.Exponential(-2.0), 1e-9);
    }

    @Test
    void triangleWaveProducesExpectedShape() {
        assertEquals(0.0, Functions.TriangleWave(0.0), 1e-9);
        assertEquals(1.0, Functions.TriangleWave(0.25), 1e-9);
    }

    @Test
    void pythagoreanAndVectorMagnitudeComputeHypotenuse() {
        assertEquals(5.0, Functions.Pythagorean(3.0, 4.0), 1e-9);
        assertEquals(5.0, Functions.VectorMagnitude(3.0, -4.0), 1e-9);
    }

    @Test
    void deltaAngleDegWrapsProperly() {
        assertEquals(20.0, Functions.DeltaAngleDeg(350.0, 10.0), 1e-9);
        assertEquals(-20.0, Functions.DeltaAngleDeg(10.0, 350.0), 1e-9);
    }

    @Test
    void deltaAngleRadiansWrapsProperly() {
        assertEquals(Math.PI / 2, Functions.DeltaAngleRadians(0.0, Math.PI / 2), 1e-9);
        assertEquals(-Math.PI, Functions.DeltaAngleRadians(3 * Math.PI / 2, Math.PI / 2), 1e-9);
    }

    @Test
    void clampFunctionsLimitValues() {
        assertEquals(5.0, Functions.Clamp(5.0, 0.0, 10.0), 1e-9);
        assertEquals(0.0, Functions.Clamp(-1.0, 0.0, 10.0), 1e-9);
        assertEquals(10.0, Functions.Clamp(15.0, 0.0, 10.0), 1e-9);

        assertEquals(10.0, Functions.ClampMin(5.0, 10.0), 1e-9);
        assertEquals(15.0, Functions.ClampMin(15.0, 10.0), 1e-9);

        assertEquals(10.0, Functions.ClampMax(15.0, 10.0), 1e-9);
        assertEquals(5.0, Functions.ClampMax(5.0, 10.0), 1e-9);
    }

    @Test
    void maxFindsLargestValue() {
        double[] values = {1.0, 5.0, 3.0};
        assertEquals(5.0, Functions.Max(values), 1e-9);
    }

    @Test
    void inRangeOfDetectsProximity() {
        assertTrue(Functions.InRangeOf(5.0, 10.0, 6.0));
        assertFalse(Functions.InRangeOf(20.0, 10.0, 6.0));
    }

    @Test
    void containsMultipleDetectsRepeatedCharacter() {
        assertEquals("l", Functions.ContainsMultiple("hello"));
        assertEquals("", Functions.ContainsMultiple("abc"));
    }

    @Test
    void containsMultipleOfDetectsRepeatedSubstring() {
        assertTrue(Functions.ContainsMultipleOf("banana", "an"));
        assertFalse(Functions.ContainsMultipleOf("apple", "pp"));
    }
}

