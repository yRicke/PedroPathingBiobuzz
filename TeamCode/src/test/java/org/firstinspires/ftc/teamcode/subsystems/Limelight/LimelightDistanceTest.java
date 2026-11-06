package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import org.junit.Test;
import org.junit.Before;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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

    @Test public void rejectsIsolatedSpikesInFortyUniqueSamples() {
        OptionalDouble result = OptionalDouble.empty();
        for (int i = 0; i < 40; i++) update(2, i + 1, i * 25_000_000L);
        for (int i = 0; i < 40; i++) {
            int index = (i * 17) % 40;
            double value = index < 10 ? 0.5 : index < 30 ? 2 : 20;
            result = update(value, i + 41, (i + 40) * 25_000_000L);
        }
        assertEquals(40, limelight.getDistanceSampleCount());
        assertEquals(2, result.getAsDouble(), 1e-9);
    }

    @Test public void repeatedOrRateLimitedFramesDoNotRecomputeOrEnterLater() {
        OptionalDouble first = update(2, 1, 0);
        assertSame(first, update(8, 1, 5_000_000));
        assertSame(first, update(8, 2, 10_000_000));
        assertSame(first, update(8, 2, 30_000_000));
        assertEquals(1, limelight.getDistanceSampleCount());
        double smoothed = update(4, 3, 50_000_000).getAsDouble();
        assertTrue(smoothed > 2 && smoothed < 3);
    }

    @Test public void expiredSamplesLeaveTheWindowWhenFreshFramesArrive() {
        update(2, 1, 0);
        update(4, 2, 25_000_000);
        double result = update(8, 3, 1_000_000_000).getAsDouble();
        assertEquals(2, limelight.getDistanceSampleCount());
        assertTrue(result > 5.8 && result < 6);
        result = update(10, 4, 1_025_000_000).getAsDouble();
        assertEquals(2, limelight.getDistanceSampleCount());
        assertTrue(result > 6 && result < 9);
    }

    @Test public void centimeterNoiseIsHeldButSustainedMovementEventuallyTracks() {
        update(2, 1, 0);
        for (int i = 1; i <= 80; i++) {
            assertEquals(2, update(i % 2 == 0 ? 1.995 : 2.005,
                    i + 1, i * 25_000_000L).getAsDouble(), 0);
        }
        assertEquals(40, limelight.getDistanceSampleCount());
        double previous = 2;
        for (int i = 81; i <= 200; i++) {
            double result = update(2.5, i + 1, i * 25_000_000L).getAsDouble();
            assertTrue(result >= previous && result <= 2.5);
            previous = result;
        }
        assertEquals(2.5, previous, Limelight.DISTANCE_DEADBAND_METERS);
        assertEquals(40, limelight.getDistanceSampleCount());
    }

    @Test public void smoothingContinuesAccumulatingChangesInsidePublishedDeadband() {
        update(2, 1, 0);
        double result = 2;
        for (int i = 1; i <= 120; i++) {
            result = update(2 + i * 0.001, i + 1, i * 25_000_000L).getAsDouble();
        }
        assertTrue(result > 2.07 && result < 2.12);
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

    @Test public void briefDetectionLossHoldsDistanceButNeverRetainsAim() {
        update(2, 1, 0);
        assertEquals(2, read(OptionalDouble.empty(), 2, 42, 25_000_000).getAsDouble(), 0);
        assertFalse(limelight.getObservation().hasTarget());
        assertEquals(1, limelight.getDistanceSampleCount());
        assertEquals(2, read(OptionalDouble.empty(), 3, 42, 200_000_000).getAsDouble(), 0);
        assertFalse(read(OptionalDouble.empty(), 3, 42, 200_000_001).isPresent());
        assertEquals(0, limelight.getDistanceSampleCount());
        assertFalse(limelight.getDistanceStatus().startsWith("Mantendo"));
        assertEquals(8, update(8, 5, 225_000_000).getAsDouble(), 0);
    }

    @Test public void recoveringFromBriefLossPreservesFilteringInsteadOfJumpingToRawValue() {
        for (int i = 0; i < 20; i++) update(2, i + 1, i * 25_000_000L);
        read(OptionalDouble.empty(), 21, 42, 500_000_000);
        assertEquals(2, update(8, 22, 550_000_000).getAsDouble(), 0);
        assertEquals(21, limelight.getDistanceSampleCount());
    }

    @Test public void invalidDistancesNeverStartAFilter() {
        int timestamp = 0;
        for (double value : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
            assertFalse(update(value, ++timestamp, timestamp * 25_000_000L).isPresent());
        }
        assertEquals(8, update(8, ++timestamp, timestamp * 25_000_000L).getAsDouble(), 0);
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
        if (value.isPresent()) {
            when(tag.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                    new Position(DistanceUnit.METER, -0.1651, 0.18255996,
                            value.getAsDouble() - 0.1427988, 0),
                    new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0)));
        }
        when(camera.getLatestResult()).thenReturn(frame);
        limelight.run(target == 38, target == 42, false, false);
        return limelight.getDistanceMeters();
    }
}
