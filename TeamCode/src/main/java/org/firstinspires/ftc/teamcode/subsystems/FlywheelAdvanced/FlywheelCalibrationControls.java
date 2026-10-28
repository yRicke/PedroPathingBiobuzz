package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced;

/** Ajuste por clique; decimos inteiros evitam acumular erro nos passos de 0,1. */
public final class FlywheelCalibrationControls {
    private static final int[] STEPS_TENTHS = {1, 10, 100, 1000, 10000};
    private final int maxTenths;
    private int targetTenths;
    private int stepIndex = 2;
    private boolean enabled;
    private boolean previousUp, previousDown, previousLeft, previousRight, previousA;

    public FlywheelCalibrationControls(double initialRpm, double maxRpm) {
        if (!Double.isFinite(initialRpm) || !Double.isFinite(maxRpm)
                || initialRpm < 0 || maxRpm <= 0 || initialRpm > maxRpm
                || maxRpm > Integer.MAX_VALUE / 10.0) {
            throw new IllegalArgumentException("Limites de RPM invalidos");
        }
        maxTenths = (int) Math.round(maxRpm * 10);
        targetTenths = (int) Math.round(initialRpm * 10);
    }

    public void update(boolean up, boolean down, boolean left, boolean right,
                       boolean a, boolean stop) {
        if (left != right) {
            if (left && !previousLeft) stepIndex = Math.max(0, stepIndex - 1);
            if (right && !previousRight) {
                stepIndex = Math.min(STEPS_TENTHS.length - 1, stepIndex + 1);
            }
        }
        if (up != down) {
            if (up && !previousUp) {
                targetTenths = (int) Math.min((long) maxTenths,
                        (long) targetTenths + STEPS_TENTHS[stepIndex]);
            }
            if (down && !previousDown) {
                targetTenths = Math.max(0, targetTenths - STEPS_TENTHS[stepIndex]);
            }
        }
        if (stop) enabled = false;
        else if (a && !previousA) enabled = !enabled;
        previousUp = up;
        previousDown = down;
        previousLeft = left;
        previousRight = right;
        previousA = a;
    }

    public double getTargetRpm() { return targetTenths / 10.0; }
    public double getStepRpm() { return STEPS_TENTHS[stepIndex] / 10.0; }
    public boolean isEnabled() { return enabled; }
    public double getCommandedRpm() { return enabled ? getTargetRpm() : 0; }
}
