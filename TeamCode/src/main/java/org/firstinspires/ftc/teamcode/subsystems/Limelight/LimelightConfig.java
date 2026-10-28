package org.firstinspires.ftc.teamcode.subsystems.Limelight;

/** Configuracao da camera e da amostragem do tdist. */
public final class LimelightConfig {
    public static final String DEVICE_NAME = "limelight";
    public static final int PIPELINE_INDEX = 8;
    public static final boolean UPSIDE_DOWN = true;
    public static final int POLL_RATE_HZ = 100;
    public static final long MAX_FRAME_AGE_MS = 150;
    public static final long DISTANCE_WINDOW_MS = 1000;
    public static final int DISTANCE_MAX_SAMPLES = 40;
    public static final int LEFT_HIVE_FIRST_TAG = 38;
    public static final int RIGHT_HIVE_FIRST_TAG = 42;
    public static final int TAGS_PER_HIVE = 4;

    private LimelightConfig() { }

    static boolean isHive(int firstTagId) {
        return firstTagId == LEFT_HIVE_FIRST_TAG || firstTagId == RIGHT_HIVE_FIRST_TAG;
    }

    static boolean containsTag(int firstTagId, int tagId) {
        return isHive(firstTagId) && tagId >= firstTagId && tagId < firstTagId + TAGS_PER_HIVE;
    }
}
