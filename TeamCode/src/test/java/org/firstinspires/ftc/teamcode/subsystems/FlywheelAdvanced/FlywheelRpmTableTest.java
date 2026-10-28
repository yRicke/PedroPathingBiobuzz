package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced;

import org.junit.Test;
import java.util.OptionalDouble;
import static org.junit.Assert.*;

public class FlywheelRpmTableTest {
    private FlywheelRpmTable table(double[][] pairs) {
        return new FlywheelRpmTable(2720, 6000, pairs);
    }

    @Test public void interpolatesAndPreservesMeasuredPointsInAnyOrder() {
        FlywheelRpmTable table = table(new double[][] {{200, 3200}, {100, 2720}, {150, 2900}});
        assertEquals(3, table.getPointCount());
        assertEquals(2720, table.calculate(OptionalDouble.of(1)), 1e-9);
        assertEquals(2810, table.calculate(OptionalDouble.of(1.25)), 1e-9);
        assertEquals(2900, table.calculate(OptionalDouble.of(1.5)), 1e-9);
        assertEquals(3050, table.calculate(OptionalDouble.of(1.75)), 1e-9);
        assertEquals(3200, table.calculate(OptionalDouble.of(2)), 1e-9);
    }

    @Test public void constructorCopiesMeasurementsSoRuntimeTableCannotChange() {
        double[][] pairs = {{100, 2720}, {200, 3200}};
        FlywheelRpmTable table = table(pairs);
        pairs[0][0] = 180;
        pairs[0][1] = 5000;
        pairs[1] = new double[] {300, 4000};
        assertEquals(2960, table.calculate(OptionalDouble.of(1.5)), 1e-9);
    }

    @Test public void clampsEndpointsAndReportsUncalibratedRange() {
        FlywheelRpmTable table = table(new double[][] {{100, 2720}, {200, 3200}});
        assertEquals(2720, table.calculate(OptionalDouble.of(0.5)), 0);
        assertEquals(3200, table.calculate(OptionalDouble.of(5)), 0);
        assertEquals(FlywheelRpmTable.Mode.LIMITED, table.getMode(OptionalDouble.of(5)));
        assertEquals(FlywheelRpmTable.Mode.CALIBRATED, table.getMode(OptionalDouble.of(2)));
    }

    @Test public void onePointCannotDefineDistanceScaling() {
        FlywheelRpmTable table = table(new double[][] {{150, 2900}, {0, 0}});
        assertEquals(1, table.getPointCount());
        assertEquals(2900, table.calculate(OptionalDouble.of(0.5)), 0);
        assertEquals(2900, table.calculate(OptionalDouble.of(5)), 0);
        assertEquals(FlywheelRpmTable.Mode.SINGLE_POINT, table.getMode(OptionalDouble.of(5)));
    }

    @Test public void noPointsOrInvalidDistanceUsesConfiguredBase() {
        FlywheelRpmTable empty = table(new double[][] {{0, 2720}, {0, 0}});
        assertEquals(2720, empty.calculate(OptionalDouble.of(2)), 0);
        assertEquals(FlywheelRpmTable.Mode.NO_CALIBRATION, empty.getMode(OptionalDouble.of(2)));
        FlywheelRpmTable measured = table(new double[][] {{150, 2900}});
        assertEquals(2720, measured.calculate(OptionalDouble.empty()), 0);
        for (double invalid : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY, Double.MAX_VALUE}) {
            assertEquals(2720, measured.calculate(OptionalDouble.of(invalid)), 0);
            assertEquals(FlywheelRpmTable.Mode.NO_DISTANCE, measured.getMode(OptionalDouble.of(invalid)));
        }
    }

    @Test public void zeroBaseDisablesAutomaticSpeed() {
        FlywheelRpmTable table = new FlywheelRpmTable(0, 6000, new double[][] {{100, 2720}});
        assertEquals(0, table.calculate(OptionalDouble.of(1)), 0);
        assertEquals(FlywheelRpmTable.Mode.STOPPED, table.getMode(OptionalDouble.of(1)));
    }

    @Test public void rejectsDuplicateMalformedAndOutOfLimitMeasurements() {
        for (double[][] pairs : new double[][][] {
                {{150, 2900}, {150, 3200}}, {{150, Double.NaN}}, {{-1, 2900}},
                {{150, 0}}, {{150, 6001}}, {{Double.NaN, 2900}}, {{150}}, {null}}) {
            assertThrows(IllegalArgumentException.class, () -> table(pairs));
        }
        assertThrows(IllegalArgumentException.class, () -> new FlywheelRpmTable(Double.NaN, 6000, new double[0][]));
    }
}
