package org.firstinspires.ftc.teamcode.subsystems.Limelight;

/** Observacao imutavel da abertura da CELL para controle de heading do lancador. */
public final class AprilTagObservation {
    private final int firstTagId;
    private final boolean visionReady;
    private final boolean targetVisible;
    private final double txDegrees;
    private final int tagsUsed;
    private final double frameTimestamp;
    private final double imageAgeSeconds;

    private AprilTagObservation(int firstTagId, boolean visionReady, boolean targetVisible,
                                double txDegrees, int tagsUsed, double frameTimestamp,
                                double imageAgeSeconds) {
        this.firstTagId = firstTagId;
        this.visionReady = visionReady;
        this.targetVisible = targetVisible;
        this.txDegrees = txDegrees;
        this.tagsUsed = tagsUsed;
        this.frameTimestamp = frameTimestamp;
        this.imageAgeSeconds = imageAgeSeconds;
    }

    public static AprilTagObservation unavailable(int firstTagId) {
        return new AprilTagObservation(firstTagId, false, false, 0, 0, Double.NaN, 0);
    }

    public static AprilTagObservation searching(int firstTagId, double frameTimestamp) {
        return new AprilTagObservation(firstTagId, true, false, 0, 0, frameTimestamp, 0);
    }

    public static AprilTagObservation detected(int firstTagId, double txDegrees,
                                               int tagsUsed, double frameTimestamp) {
        return detected(firstTagId, txDegrees, tagsUsed, frameTimestamp, 0);
    }

    /** Idade da imagem na primeira leitura: captura + processamento + transporte, em segundos. */
    public static AprilTagObservation detected(int firstTagId, double txDegrees,
                                               int tagsUsed, double frameTimestamp,
                                               double imageAgeSeconds) {
        if (!Double.isFinite(txDegrees) || !Double.isFinite(frameTimestamp)
                || frameTimestamp < 0 || tagsUsed <= 0 || !Double.isFinite(imageAgeSeconds)
                || imageAgeSeconds < 0 || imageAgeSeconds > 0.3) return unavailable(firstTagId);
        return new AprilTagObservation(firstTagId, true, true, txDegrees, tagsUsed,
                frameTimestamp, imageAgeSeconds);
    }

    public int getFirstTagId() { return firstTagId; }
    public boolean isVisionReady() { return visionReady; }
    public boolean hasTarget() { return targetVisible; }
    public double getTxDegrees() { return txDegrees; }
    public int getTagsUsed() { return tagsUsed; }
    public double getFrameTimestamp() { return frameTimestamp; }
    public double getImageAgeSeconds() { return imageAgeSeconds; }
}
