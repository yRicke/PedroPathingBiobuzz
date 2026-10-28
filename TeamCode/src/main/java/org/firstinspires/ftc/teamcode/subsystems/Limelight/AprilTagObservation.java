package org.firstinspires.ftc.teamcode.subsystems.Limelight;

/** Observacao imutavel do centro do hive para controle de heading. */
public final class AprilTagObservation {
    private final int firstTagId;
    private final boolean visionReady;
    private final boolean targetVisible;
    private final double txDegrees;
    private final int tagsUsed;
    private final double frameTimestamp;

    private AprilTagObservation(int firstTagId, boolean visionReady, boolean targetVisible,
                                double txDegrees, int tagsUsed, double frameTimestamp) {
        this.firstTagId = firstTagId;
        this.visionReady = visionReady;
        this.targetVisible = targetVisible;
        this.txDegrees = txDegrees;
        this.tagsUsed = tagsUsed;
        this.frameTimestamp = frameTimestamp;
    }

    public static AprilTagObservation unavailable(int firstTagId) {
        return new AprilTagObservation(firstTagId, false, false, 0, 0, Double.NaN);
    }

    public static AprilTagObservation searching(int firstTagId, double frameTimestamp) {
        return new AprilTagObservation(firstTagId, true, false, 0, 0, frameTimestamp);
    }

    public static AprilTagObservation detected(int firstTagId, double txDegrees,
                                               int tagsUsed, double frameTimestamp) {
        if (!Double.isFinite(txDegrees) || !Double.isFinite(frameTimestamp)
                || frameTimestamp < 0 || tagsUsed <= 0) return unavailable(firstTagId);
        return new AprilTagObservation(firstTagId, true, true, txDegrees, tagsUsed, frameTimestamp);
    }

    public int getFirstTagId() { return firstTagId; }
    public boolean isVisionReady() { return visionReady; }
    public boolean hasTarget() { return targetVisible; }
    public double getTxDegrees() { return txDegrees; }
    public int getTagsUsed() { return tagsUsed; }
    public double getFrameTimestamp() { return frameTimestamp; }
}
