package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import org.junit.Test;
import java.util.OptionalDouble;
import static org.junit.Assert.*;

public class DistanceFilterTest {
    private final DistanceFilter filter = new DistanceFilter();

    @Test public void usesCentralTwentyOfFortyUniqueSamples() {
        OptionalDouble result = OptionalDouble.empty();
        for (int i = 0; i < 40; i++) {
            int index = (i * 17) % 40;
            double value = index < 10 ? 0.5 : index < 25 ? 2 : index < 30 ? 6 : 20;
            result = update(value, i + 1, i * 25_000_000L);
        }
        assertEquals(40, filter.sampleCount());
        assertEquals(3, result.getAsDouble(), 1e-9);
    }

    @Test public void repeatedOrRateLimitedFramesDoNotRecomputeOrEnterLater() {
        OptionalDouble first = update(2, 1, 0);
        assertSame(first, update(8, 1, 5_000_000));
        assertSame(first, update(8, 2, 10_000_000));
        assertSame(first, update(8, 2, 30_000_000));
        assertEquals(1, filter.sampleCount());
        assertEquals(3, update(4, 3, 50_000_000).getAsDouble(), 0);
    }

    @Test public void expiredSamplesRecomputeEvenWithoutANewFrame() {
        update(2, 1, 0);
        update(4, 2, 25_000_000);
        assertEquals(4, update(4, 2, 1_000_000_000).getAsDouble(), 0);
        assertFalse(update(4, 2, 1_025_000_000).isPresent());
    }

    @Test public void switchingTargetsClearsHistoryEvenInSameFrame() {
        update(2, 1, 0);
        OptionalDouble result = filter.update(OptionalDouble.of(8), 1, 38, 1);
        assertEquals(8, result.getAsDouble(), 0);
        assertEquals(1, filter.sampleCount());
    }

    @Test public void cameraTimestampRestartClearsPreviousMeasurements() {
        update(2, 100, 0);
        update(4, 101, 25_000_000);
        assertEquals(8, update(8, 1, 50_000_000).getAsDouble(), 0);
        assertEquals(1, filter.sampleCount());
    }

    @Test public void unavailableAndNonFiniteDistancesClearTheWindow() {
        update(2, 1, 0);
        assertFalse(filter.update(OptionalDouble.empty(), 2, 42, 1).isPresent());
        assertEquals(0, filter.sampleCount());
        for (double value : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
            assertFalse(update(value, 3, 25_000_000).isPresent());
        }
        assertEquals(8, update(8, 4, 50_000_000).getAsDouble(), 0);
    }

    private OptionalDouble update(double value, double timestamp, long now) {
        return filter.update(OptionalDouble.of(value), timestamp, 42, now);
    }
}
