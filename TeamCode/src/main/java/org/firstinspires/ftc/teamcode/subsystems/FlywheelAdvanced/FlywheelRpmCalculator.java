package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced;

import java.util.OptionalDouble;

/** Converte a distancia da Limelight em RPM usando uma referencia ajustavel. */
public final class FlywheelRpmCalculator {
    private FlywheelRpmCalculator() { }

    /** Sem distancia ou referencia valida, usa o RPM base; base invalida desliga. */
    public static double calculate(OptionalDouble distanceMeters, double baseRpm,
                                   double referenceDistanceCm) {
        if (!Double.isFinite(baseRpm) || baseRpm <= 0) {
            return 0;
        }
        if (!distanceMeters.isPresent() || !Double.isFinite(referenceDistanceCm)
                || referenceDistanceCm <= 0) {
            return baseRpm;
        }

        double distance = distanceMeters.getAsDouble();
        if (!Double.isFinite(distance) || distance <= 0) {
            return baseRpm;
        }
        double rpm = baseRpm * (distance * 100 / referenceDistanceCm);
        return Double.isFinite(rpm) && rpm > 0 ? rpm : baseRpm;
    }
}
