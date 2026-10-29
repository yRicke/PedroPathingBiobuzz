package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.OptionalDouble;

/** Tabela imutavel: valida e ordena pares {distancia ao centro em cm, RPM}. */
public final class FlywheelRpmTable {
    public enum Mode {
        STOPPED("Desligada"), NO_DISTANCE("Sem distancia 3D: RPM base"),
        NO_CALIBRATION("Sem pares: RPM base"), SINGLE_POINT("Um par: RPM fixa"),
        CALIBRATED("Interpolacao distancia 3D"), LIMITED("Fora da faixa: RPM limitada ao extremo");
        private final String description;
        Mode(String description) { this.description = description; }
        public String getDescription() { return description; }
    }

    private final double baseRpm;
    private final Point[] points;

    public FlywheelRpmTable(double baseRpm, double maxRpm, double[][] pairs) {
        if (!Double.isFinite(maxRpm) || maxRpm <= 0 || !Double.isFinite(baseRpm)
                || baseRpm < 0 || baseRpm > maxRpm || pairs == null) {
            throw new IllegalArgumentException("Configuracao da tabela de RPM invalida");
        }
        this.baseRpm = baseRpm;
        List<Point> valid = new ArrayList<>();
        for (double[] pair : pairs) {
            if (pair == null || pair.length != 2) {
                throw new IllegalArgumentException("Cada par precisa de distancia (cm) e RPM");
            }
            if (pair[0] == 0) continue;
            if (!Double.isFinite(pair[0]) || pair[0] < 0 || !Double.isFinite(pair[1])
                    || pair[1] <= 0 || pair[1] > maxRpm) {
                throw new IllegalArgumentException("Par distancia/RPM invalido");
            }
            valid.add(new Point(pair[0], pair[1]));
        }
        valid.sort(Comparator.comparingDouble(p -> p.distanceCm));
        points = valid.toArray(new Point[0]);
        for (int i = 1; i < points.length; i++) {
            if (points[i - 1].distanceCm == points[i].distanceCm) {
                throw new IllegalArgumentException("Distancia de calibracao duplicada");
            }
        }
    }

    public double calculate(OptionalDouble distanceMeters) {
        if (baseRpm == 0) return 0;
        double cm = centimeters(distanceMeters);
        if (!Double.isFinite(cm) || points.length == 0) return baseRpm;
        if (cm <= points[0].distanceCm) return points[0].rpm;
        for (int i = 1; i < points.length; i++) {
            Point low = points[i - 1], high = points[i];
            if (cm <= high.distanceCm) {
                double fraction = (cm - low.distanceCm) / (high.distanceCm - low.distanceCm);
                return low.rpm * (1 - fraction) + high.rpm * fraction;
            }
        }
        return points[points.length - 1].rpm;
    }

    public Mode getMode(OptionalDouble distanceMeters) {
        if (baseRpm == 0) return Mode.STOPPED;
        double cm = centimeters(distanceMeters);
        if (!Double.isFinite(cm)) return Mode.NO_DISTANCE;
        if (points.length == 0) return Mode.NO_CALIBRATION;
        if (points.length == 1) return Mode.SINGLE_POINT;
        return cm < points[0].distanceCm || cm > points[points.length - 1].distanceCm
                ? Mode.LIMITED : Mode.CALIBRATED;
    }

    public int getPointCount() { return points.length; }

    private static double centimeters(OptionalDouble distanceMeters) {
        if (!distanceMeters.isPresent()) return Double.NaN;
        double cm = distanceMeters.getAsDouble() * 100;
        return Double.isFinite(cm) && cm > 0 ? cm : Double.NaN;
    }

    private static final class Point {
        final double distanceCm, rpm;
        Point(double distanceCm, double rpm) { this.distanceCm = distanceCm; this.rpm = rpm; }
    }
}
