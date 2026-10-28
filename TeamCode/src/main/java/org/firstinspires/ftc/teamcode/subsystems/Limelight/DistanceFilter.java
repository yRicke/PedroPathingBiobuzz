package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;
import java.util.OptionalDouble;
import java.util.concurrent.TimeUnit;

/** Media aparada por janela temporal, sem duplicar frames ou misturar alvos. */
final class DistanceFilter {
    private final int maxSamples;
    private final long windowNanos;
    private final long sampleIntervalNanos;
    private final Deque<Sample> samples = new ArrayDeque<>();
    private int targetId;
    private double lastFrameTimestamp = Double.NaN;
    private long lastSampleNanos;
    private OptionalDouble filtered = OptionalDouble.empty();

    DistanceFilter(int maxSamples, long windowNanos) {
        if (maxSamples <= 0 || windowNanos < maxSamples) {
            throw new IllegalArgumentException("Janela de distancia invalida");
        }
        this.maxSamples = maxSamples;
        this.windowNanos = windowNanos;
        sampleIntervalNanos = windowNanos / maxSamples;
    }

    DistanceFilter() {
        this(LimelightConfig.DISTANCE_MAX_SAMPLES,
                TimeUnit.MILLISECONDS.toNanos(LimelightConfig.DISTANCE_WINDOW_MS));
    }

    OptionalDouble update(OptionalDouble distanceMeters, double timestamp, int target, long now) {
        if (!distanceMeters.isPresent() || !Double.isFinite(distanceMeters.getAsDouble())
                || distanceMeters.getAsDouble() <= 0 || !Double.isFinite(timestamp) || timestamp < 0) {
            clear();
            return filtered;
        }
        if (targetId != target || timestamp < lastFrameTimestamp) {
            clear();
            targetId = target;
        }
        boolean changed = false;
        while (!samples.isEmpty() && now - samples.peekFirst().nanos >= windowNanos) {
            samples.removeFirst();
            changed = true;
        }
        if (timestamp != lastFrameTimestamp) {
            lastFrameTimestamp = timestamp;
            if (samples.isEmpty() || now - lastSampleNanos >= sampleIntervalNanos) {
                samples.addLast(new Sample(now, distanceMeters.getAsDouble()));
                lastSampleNanos = now;
                if (samples.size() > maxSamples) samples.removeFirst();
                changed = true;
            }
        }
        if (changed) filtered = trimmedMean();
        return filtered;
    }

    int sampleCount() { return samples.size(); }

    void clear() {
        samples.clear();
        filtered = OptionalDouble.empty();
        lastFrameTimestamp = Double.NaN;
        lastSampleNanos = 0;
    }

    private OptionalDouble trimmedMean() {
        if (samples.isEmpty()) return OptionalDouble.empty();
        double[] sorted = new double[samples.size()];
        int index = 0;
        for (Sample sample : samples) sorted[index++] = sample.distanceMeters;
        Arrays.sort(sorted);
        int trim = sorted.length / 4;
        int count = sorted.length - 2 * trim;
        double mean = 0;
        for (int i = trim; i < sorted.length - trim; i++) mean += sorted[i] / count;
        return OptionalDouble.of(mean);
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
