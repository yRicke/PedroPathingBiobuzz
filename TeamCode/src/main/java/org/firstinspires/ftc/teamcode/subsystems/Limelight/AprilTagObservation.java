package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.OptionalDouble;

/** Centro estimado do hive selecionado, com disponibilidade separada da deteccao. */
public final class AprilTagObservation {
    private final int firstTagId;
    private final boolean visionReady;
    private final boolean targetVisible;
    private final double txDegrees;
    private final double distanceMeters;
    private final int tagsUsed;
    private final double frameTimestamp;

    private AprilTagObservation(int firstTagId, boolean visionReady, boolean targetVisible,
                                double txDegrees, double distanceMeters, int tagsUsed,
                                double frameTimestamp) {
        this.firstTagId = firstTagId;
        this.visionReady = visionReady;
        this.targetVisible = targetVisible;
        this.txDegrees = txDegrees;
        this.distanceMeters = distanceMeters;
        this.tagsUsed = tagsUsed;
        this.frameTimestamp = frameTimestamp;
    }

    public static AprilTagObservation unavailable(int firstTagId) {
        return new AprilTagObservation(firstTagId, false, false, 0, Double.NaN, 0, Double.NaN);
    }

    public static AprilTagObservation searching(int firstTagId, double frameTimestamp) {
        return new AprilTagObservation(firstTagId, true, false, 0, Double.NaN, 0, frameTimestamp);
    }

    public static AprilTagObservation detected(int firstTagId, double txDegrees,
                                               double distanceMeters, int tagsUsed,
                                               double frameTimestamp) {
        if (!Double.isFinite(txDegrees) || !Double.isFinite(frameTimestamp)
                || !Double.isFinite(distanceMeters) || distanceMeters <= 0 || tagsUsed <= 0) {
            return unavailable(firstTagId);
        }
        return new AprilTagObservation(firstTagId, true, true, txDegrees,
                distanceMeters, tagsUsed, frameTimestamp);
    }

    /** Identifica o hive pela primeira tag: 38 (esquerdo) ou 42 (direito). */
    public int getFirstTagId() { return firstTagId; }
    public boolean isVisionReady() { return visionReady; }
    public boolean hasTarget() { return targetVisible; }
    public double getTxDegrees() { return txDegrees; }
    public OptionalDouble getDistanceMeters() {
        return targetVisible ? OptionalDouble.of(distanceMeters) : OptionalDouble.empty();
    }
    public int getTagsUsed() { return tagsUsed; }
    public double getFrameTimestamp() { return frameTimestamp; }
}
