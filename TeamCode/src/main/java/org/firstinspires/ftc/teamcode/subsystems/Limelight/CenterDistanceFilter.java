package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;
import java.util.OptionalDouble;
import java.util.concurrent.TimeUnit;

/** Media aparada da distancia ao centro, sem repetir frames nem misturar hives. */
final class CenterDistanceFilter {
    private static final int MAX_SAMPLES = 40;
    private static final long WINDOW_NANOS = TimeUnit.SECONDS.toNanos(1);
    private static final long SAMPLE_INTERVAL_NANOS = WINDOW_NANOS / MAX_SAMPLES;

    private final Deque<Sample> samples = new ArrayDeque<>();
    private int firstTagId;
    private double lastFrameTimestamp = Double.NaN;
    private long lastSampleNanos;

    OptionalDouble update(AprilTagObservation observation, long now) {
        if (!observation.isVisionReady() || !observation.hasTarget()) {
            clear();
            return OptionalDouble.empty();
        }
        if (firstTagId != observation.getFirstTagId()) {
            clear();
            firstTagId = observation.getFirstTagId();
        }

        while (!samples.isEmpty() && now - samples.peekFirst().nanos >= WINDOW_NANOS) {
            samples.removeFirst();
        }

        if (observation.getFrameTimestamp() != lastFrameTimestamp) {
            // Mesmo um frame nao amostrado nao pode entrar depois como se fosse novo.
            lastFrameTimestamp = observation.getFrameTimestamp();
            if (samples.isEmpty() || now - lastSampleNanos >= SAMPLE_INTERVAL_NANOS) {
                samples.addLast(new Sample(now, observation.getDistanceMeters().getAsDouble()));
                lastSampleNanos = now;
                while (samples.size() > MAX_SAMPLES) {
                    samples.removeFirst();
                }
            }
        }

        if (samples.isEmpty()) {
            return OptionalDouble.empty();
        }
        double[] sorted = new double[samples.size()];
        int index = 0;
        for (Sample sample : samples) {
            sorted[index++] = sample.distanceMeters;
        }
        Arrays.sort(sorted);
        int trim = sorted.length / 4;
        int centralCount = sorted.length - 2 * trim;
        double mean = 0;
        for (int i = trim; i < sorted.length - trim; i++) {
            mean += sorted[i] / centralCount;
        }
        return OptionalDouble.of(mean);
    }

    int sampleCount() {
        return samples.size();
    }

    void clear() {
        samples.clear();
        lastFrameTimestamp = Double.NaN;
        lastSampleNanos = 0;
    }

    private static final class Sample {
        final long nanos;
        final double distanceMeters;

        Sample(long nanos, double distanceMeters) {
            this.nanos = nanos;
            this.distanceMeters = distanceMeters;
        }
    }
}
