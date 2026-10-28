package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import org.junit.Test;
import org.junit.Before;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.Collections;
import java.util.concurrent.atomic.AtomicLong;
import static org.mockito.Mockito.*;
import java.util.OptionalDouble;
import static org.junit.Assert.*;

public class LimelightDistanceTest {
    private final AtomicLong clock = new AtomicLong();
    private Limelight3A camera;
    private Limelight limelight;

    @Before public void setup() {
        camera = mock(Limelight3A.class);
        when(camera.isConnected()).thenReturn(true);
        when(camera.isRunning()).thenReturn(true);
        limelight = new Limelight(camera, null, false, 8, clock::get);
    }

    @Test public void usesCentralTwentyOfFortyUniqueSamples() {
        OptionalDouble result = OptionalDouble.empty();
        for (int i = 0; i < 40; i++) {
            int index = (i * 17) % 40;
            double value = index < 10 ? 0.5 : index < 25 ? 2 : index < 30 ? 6 : 20;
            result = update(value, i + 1, i * 25_000_000L);
        }
        assertEquals(40, limelight.getDistanceSampleCount());
        assertEquals(3, result.getAsDouble(), 1e-9);
    }

    @Test public void repeatedOrRateLimitedFramesDoNotRecomputeOrEnterLater() {
        OptionalDouble first = update(2, 1, 0);
        assertSame(first, update(8, 1, 5_000_000));
        assertSame(first, update(8, 2, 10_000_000));
        assertSame(first, update(8, 2, 30_000_000));
        assertEquals(1, limelight.getDistanceSampleCount());
        assertEquals(3, update(4, 3, 50_000_000).getAsDouble(), 0);
    }

    @Test public void expiredSamplesLeaveTheWindowWhenFreshFramesArrive() {
        update(2, 1, 0);
        update(4, 2, 25_000_000);
        assertEquals(6, update(8, 3, 1_000_000_000).getAsDouble(), 0);
        assertEquals(9, update(10, 4, 1_025_000_000).getAsDouble(), 0);
    }

    @Test public void circularWindowRetainsOnlyFortyRecentFrames() {
        OptionalDouble result = OptionalDouble.empty();
        for (int i = 0; i < 80; i++) result = update(i + 1, i + 1, i * 25_000_000L);
        assertEquals(40, limelight.getDistanceSampleCount());
        assertEquals(60.5, result.getAsDouble(), 1e-9);
    }

    @Test public void switchingTargetsClearsHistoryEvenInSameFrame() {
        update(2, 1, 0);
        OptionalDouble result = read(OptionalDouble.of(8), 1, 38, 1);
        assertEquals(8, result.getAsDouble(), 0);
        assertEquals(1, limelight.getDistanceSampleCount());
    }

    @Test public void cameraTimestampRestartClearsPreviousMeasurements() {
        update(2, 100, 0);
        update(4, 101, 25_000_000);
        assertEquals(8, update(8, 1, 50_000_000).getAsDouble(), 0);
        assertEquals(1, limelight.getDistanceSampleCount());
    }

    @Test public void unavailableAndNonFiniteDistancesClearTheWindow() {
        update(2, 1, 0);
        assertFalse(read(OptionalDouble.empty(), 2, 42, 1).isPresent());
        assertEquals(0, limelight.getDistanceSampleCount());
        for (double value : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
            assertFalse(update(value, 3, 25_000_000).isPresent());
        }
        assertEquals(8, update(8, 4, 50_000_000).getAsDouble(), 0);
    }

    private OptionalDouble update(double value, double timestamp, long now) {
        return read(OptionalDouble.of(value), timestamp, 42, now);
    }

    private OptionalDouble read(OptionalDouble value, double timestamp, int target, long now) {
        clock.set(now);
        LLResult frame = mock(LLResult.class);
        FiducialResult tag = mock(FiducialResult.class);
        when(tag.getFiducialId()).thenReturn(target);
        when(frame.getTimestamp()).thenReturn(timestamp);
        when(frame.getPipelineIndex()).thenReturn(8);
        when(frame.isValid()).thenReturn(true);
        when(frame.getFiducialResults()).thenReturn(Collections.singletonList(tag));
        when(frame.toString()).thenReturn(value.isPresent()
                ? "{\"tdist\":" + value.getAsDouble() + "}" : "{}");
        when(camera.getLatestResult()).thenReturn(frame);
        limelight.run(target == 38, target == 42);
        return limelight.getDistanceMeters();
    }
}
