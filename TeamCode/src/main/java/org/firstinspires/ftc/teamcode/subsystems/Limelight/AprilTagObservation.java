package org.firstinspires.ftc.teamcode.subsystems.Limelight;

/** Leitura imutavel da tag selecionada, com disponibilidade separada da deteccao. */
public final class AprilTagObservation {
    private final int tagId;
    private final boolean visionReady;
    private final boolean targetVisible;
    private final double txDegrees;
    private final double frameTimestamp;

    private AprilTagObservation(int tagId, boolean visionReady, boolean targetVisible,
                                double txDegrees, double frameTimestamp) {
        this.tagId = tagId;
        this.visionReady = visionReady;
        this.targetVisible = targetVisible;
        this.txDegrees = txDegrees;
        this.frameTimestamp = frameTimestamp;
    }

    public static AprilTagObservation unavailable(int tagId) {
        return new AprilTagObservation(tagId, false, false, 0, Double.NaN);
    }

    public static AprilTagObservation searching(int tagId, double frameTimestamp) {
        return new AprilTagObservation(tagId, true, false, 0, frameTimestamp);
    }

    public static AprilTagObservation detected(int tagId, double txDegrees, double frameTimestamp) {
        if (!Double.isFinite(txDegrees) || !Double.isFinite(frameTimestamp)) {
            return unavailable(tagId);
        }
        return new AprilTagObservation(tagId, true, true, txDegrees, frameTimestamp);
    }

    public int getTagId() { return tagId; }
    public boolean isVisionReady() { return visionReady; }
    public boolean hasTarget() { return targetVisible; }
    public double getTxDegrees() { return txDegrees; }
    public double getFrameTimestamp() { return frameTimestamp; }
}
