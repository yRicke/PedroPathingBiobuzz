package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.pedropathing.utils.Angle;

/** Referencia global da visao: filtra imagens novas e evita cacar ruido perto do centro. */
final class VisionHeadingTarget {
    static final double FILTER_GAIN = 0.25;
    static final double MAX_STEP_DEGREES = 1.0;
    static final double SETTLE_DEGREES = 1.0;
    static final double RESTART_DEGREES = 1.5;

    private boolean initialized;
    private boolean settled;
    private double heading;

    /** O chamador garante uma unica atualizacao por timestamp da imagem. */
    double update(double measuredHeading) {
        if (!initialized) {
            heading = Angle.normalizeSigned(measuredHeading);
            initialized = true;
        } else {
            double innovation = Angle.normalizeSigned(measuredHeading - heading);
            double maxStep = Math.toRadians(MAX_STEP_DEGREES);
            double step = Math.max(-maxStep, Math.min(maxStep, FILTER_GAIN * innovation));
            heading = Angle.normalizeSigned(heading + step);
        }
        return heading;
    }

    double error(double currentHeading) {
        return Angle.normalizeSigned(heading - currentHeading);
    }

    boolean settled(double currentHeading, double angularSpeed) {
        if (!initialized) return false;
        double error = Math.abs(error(currentHeading));
        if (settled) {
            if (error >= Math.toRadians(RESTART_DEGREES)) settled = false;
        } else if (error <= Math.toRadians(SETTLE_DEGREES)
                && Math.abs(angularSpeed) <= PedroDrive.AIM_SETTLED_SPEED_RAD_S) {
            settled = true;
        }
        return settled;
    }

    void reset() {
        initialized = settled = false;
    }
}
