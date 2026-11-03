package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.pedropathing.utils.Angle;

/** Historico local de heading para associar cada imagem a orientacao na captura. */
final class HeadingHistory {
    private static final int CAPACITY = 256;
    private final long[] times = new long[CAPACITY];
    private final double[] headings = new double[CAPACITY];
    private int next;
    private int size;

    void record(long now, double heading) {
        if (!Double.isFinite(heading)) return;
        if (size > 0) {
            int latest = (next + CAPACITY - 1) % CAPACITY;
            if (now == times[latest]) {
                headings[latest] = heading;
                return;
            }
            if (now < times[latest]) clear();
        }
        times[next] = now;
        headings[next] = heading;
        next = (next + 1) % CAPACITY;
        size = Math.min(size + 1, CAPACITY);
    }

    double at(long captureTime) {
        if (size == 0) return Double.NaN;
        int latest = (next + CAPACITY - 1) % CAPACITY;
        if (captureTime > times[latest]) return Double.NaN;
        for (int i = 0; i < size; i++) {
            int older = (latest - i + CAPACITY) % CAPACITY;
            if (times[older] > captureTime) continue;
            if (times[older] == captureTime) return headings[older];
            int newer = (older + 1) % CAPACITY;
            double fraction = (double) (captureTime - times[older]) / (times[newer] - times[older]);
            return Angle.normalizeSigned(headings[older]
                    + fraction * Angle.normalizeSigned(headings[newer] - headings[older]));
        }
        return Double.NaN;
    }

    void clear() {
        next = size = 0;
    }
}
