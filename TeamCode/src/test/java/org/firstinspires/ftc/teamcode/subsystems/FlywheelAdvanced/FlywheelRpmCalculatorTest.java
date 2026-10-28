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
    public void onePointDoesNotInventHalfRpmAtHalfTheDistance() {
        assertEquals(2700, FlywheelRpmCalculator.calculate(
                OptionalDouble.of(1.54), 2700, 308), EPSILON);
    }

    @Test
    public void onePointKeepsRpmConstantFartherAway() {
        assertEquals(2700, FlywheelRpmCalculator.calculate(
                OptionalDouble.of(4.62), 2700, 308), EPSILON);
    }

    @Test
    public void changingBaseRpmChangesTheConstantTargetWithOnePoint() {
        assertEquals(3000, FlywheelRpmCalculator.calculate(
                OptionalDouble.of(3.08), 3000, 308), EPSILON);
        assertEquals(3000, FlywheelRpmCalculator.calculate(
                OptionalDouble.of(1.54), 3000, 308), EPSILON);
    }

    @Test
    public void referenceDistanceIsConfigurableInCentimeters() {
        assertEquals(2700, FlywheelRpmCalculator.calculate(
                OptionalDouble.of(4), 2700, 400), EPSILON);
        assertEquals(2700, FlywheelRpmCalculator.calculate(
                OptionalDouble.of(2), 2700, 400), EPSILON);
    }

    @Test
    public void missingDistanceReturnsToBaseRatherThanKeepingPreviousTarget() {
        assertEquals(2700, FlywheelRpmCalculator.calculate(
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

    @Test
    public void interpolatesBetweenThreeUnsortedMeasuredPairsInCentimeters() {
        double[][] pairs = {{200, 3200}, {100, 2720}, {150, 2900}};
        assertEquals(2720, FlywheelRpmCalculator.calculate(OptionalDouble.of(1), 2720, pairs), EPSILON);
        assertEquals(2810, FlywheelRpmCalculator.calculate(OptionalDouble.of(1.25), 2720, pairs), EPSILON);
        assertEquals(3050, FlywheelRpmCalculator.calculate(OptionalDouble.of(1.75), 2720, pairs), EPSILON);
        assertEquals(3200, FlywheelRpmCalculator.calculate(OptionalDouble.of(2), 2720, pairs), EPSILON);
    }

    @Test
    public void clampsToEndpointsInsteadOfExtrapolatingAnUncalibratedShot() {
        double[][] pairs = {{100, 2720}, {200, 3200}};
        assertEquals(2720, FlywheelRpmCalculator.calculate(OptionalDouble.of(0.5), 2720, pairs), EPSILON);
        assertEquals(3200, FlywheelRpmCalculator.calculate(OptionalDouble.of(5), 2720, pairs), EPSILON);
        org.junit.Assert.assertFalse(FlywheelRpmCalculator.isInCalibratedRange(OptionalDouble.of(5), pairs));
    }

    @Test
    public void zeroDistanceDisablesAnUnmeasuredPairAndMissingVisionUsesBase() {
        double[][] pairs = {{0, 2720}, {150, 2900}, {0, 0}};
        assertEquals(1, FlywheelRpmCalculator.pointCount(pairs));
        assertEquals(2900, FlywheelRpmCalculator.calculate(OptionalDouble.of(2), 2720, pairs), EPSILON);
        assertEquals(2720, FlywheelRpmCalculator.calculate(OptionalDouble.empty(), 2720, pairs), EPSILON);
        assertEquals(2720, FlywheelRpmCalculator.calculate(OptionalDouble.of(2), 2720,
                new double[][] {{0, 2720}, {0, 0}}), EPSILON);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsDuplicateDistancesRatherThanSilentlyPickingOneRpm() {
        FlywheelRpmCalculator.pointCount(new double[][] {{150, 2900}, {150, 3200}});
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsNonFiniteRpmInAnEnabledPair() {
        FlywheelRpmCalculator.pointCount(new double[][] {{150, Double.NaN}});
    }
}
