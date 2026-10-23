package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced;

import org.junit.Test;

import java.util.OptionalDouble;

import static org.junit.Assert.assertEquals;

public class FlywheelRpmCalculatorTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void referenceDistanceInMetersProducesBaseRpm() {
        assertEquals(2700, FlywheelRpmCalculator.calculate(
                OptionalDouble.of(3.08), 2700, 308), EPSILON);
    }

    @Test
    public void halfReferenceDistanceProducesHalfBaseRpm() {
        assertEquals(1350, FlywheelRpmCalculator.calculate(
                OptionalDouble.of(1.54), 2700, 308), EPSILON);
    }

    @Test
    public void fartherDistanceIncreasesRpmProportionally() {
        assertEquals(4050, FlywheelRpmCalculator.calculate(
                OptionalDouble.of(4.62), 2700, 308), EPSILON);
    }

    @Test
    public void changingBaseRpmChangesTheReferenceAndScaledTarget() {
        assertEquals(3000, FlywheelRpmCalculator.calculate(
                OptionalDouble.of(3.08), 3000, 308), EPSILON);
        assertEquals(1500, FlywheelRpmCalculator.calculate(
                OptionalDouble.of(1.54), 3000, 308), EPSILON);
    }

    @Test
    public void referenceDistanceIsConfigurableInCentimeters() {
        assertEquals(2700, FlywheelRpmCalculator.calculate(
                OptionalDouble.of(4), 2700, 400), EPSILON);
        assertEquals(1350, FlywheelRpmCalculator.calculate(
                OptionalDouble.of(2), 2700, 400), EPSILON);
    }

    @Test
    public void missingDistanceReturnsToBaseRatherThanKeepingPreviousTarget() {
        assertEquals(4050, FlywheelRpmCalculator.calculate(
                OptionalDouble.of(4.62), 2700, 308), EPSILON);
        assertEquals(2700, FlywheelRpmCalculator.calculate(
                OptionalDouble.empty(), 2700, 308), EPSILON);
        assertEquals(3000, FlywheelRpmCalculator.calculate(
                OptionalDouble.empty(), 3000, 308), EPSILON);
    }

    @Test
    public void invalidDistanceUsesBaseRpm() {
        double[] invalid = {0, -1, Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY, Double.MAX_VALUE};
        for (double distance : invalid) {
            assertEquals("Distance " + distance, 2700, FlywheelRpmCalculator.calculate(
                    OptionalDouble.of(distance), 2700, 308), EPSILON);
        }
    }

    @Test
    public void invalidReferenceDistanceUsesBaseRpm() {
        double[] invalid = {0, -1, Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY};
        for (double reference : invalid) {
            assertEquals("Reference " + reference, 2700, FlywheelRpmCalculator.calculate(
                    OptionalDouble.of(3.08), 2700, reference), EPSILON);
        }
    }

    @Test
    public void invalidBaseRpmProducesStoppedTarget() {
        double[] invalid = {0, -1, Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY};
        for (double baseRpm : invalid) {
            assertEquals("Base RPM " + baseRpm, 0, FlywheelRpmCalculator.calculate(
                    OptionalDouble.of(3.08), baseRpm, 308), EPSILON);
            assertEquals("Base RPM " + baseRpm, 0, FlywheelRpmCalculator.calculate(
                    OptionalDouble.empty(), baseRpm, 308), EPSILON);
        }
    }
}
