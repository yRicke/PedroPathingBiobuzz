package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced;

import java.util.Map;
import java.util.OptionalDouble;
import java.util.TreeMap;

/** Interpola pares experimentais {tdist em cm, RPM}; limita aos extremos calibrados. */
public final class FlywheelRpmCalculator {
    private FlywheelRpmCalculator() { }

    /** Um unico par nao determina uma curva: usa RPM constante. */
    public static double calculate(OptionalDouble distanceMeters, double baseRpm,
                                   double referenceDistanceCm) {
        if (!Double.isFinite(referenceDistanceCm) || referenceDistanceCm <= 0) {
            return Double.isFinite(baseRpm) && baseRpm > 0 ? baseRpm : 0;
        }
        return calculate(distanceMeters, baseRpm,
                new double[][] {{referenceDistanceCm, baseRpm}});
    }

    /**
     * Pares com distancia zero estao desativados. Sem leitura/pares usa baseRpm;
     * um par mantem sua RPM fixa. Com 2+ pares interpola, sem extrapolar.
     */
    public static double calculate(OptionalDouble distanceMeters, double baseRpm,
                                   double[][] distanceRpmPairs) {
        if (!Double.isFinite(baseRpm) || baseRpm <= 0) return 0;
        TreeMap<Double, Double> points = points(distanceRpmPairs);
        if (points.isEmpty() || !distanceMeters.isPresent()) return baseRpm;
        double distanceCm = distanceMeters.getAsDouble() * 100;
        if (!Double.isFinite(distanceCm) || distanceCm <= 0) return baseRpm;
        Map.Entry<Double, Double> low = points.floorEntry(distanceCm);
        Map.Entry<Double, Double> high = points.ceilingEntry(distanceCm);
        if (low == null) return points.firstEntry().getValue();
        if (high == null) return points.lastEntry().getValue();
        if (low.getKey().equals(high.getKey())) return low.getValue();
        double fraction = (distanceCm - low.getKey()) / (high.getKey() - low.getKey());
        return low.getValue() * (1 - fraction) + high.getValue() * fraction;
    }

    public static int pointCount(double[][] distanceRpmPairs) {
        return points(distanceRpmPairs).size();
    }

    public static boolean isInCalibratedRange(OptionalDouble distanceMeters,
                                             double[][] distanceRpmPairs) {
        TreeMap<Double, Double> points = points(distanceRpmPairs);
        if (points.isEmpty() || !distanceMeters.isPresent()) return false;
        double distanceCm = distanceMeters.getAsDouble() * 100;
        return Double.isFinite(distanceCm) && distanceCm >= points.firstKey()
                && distanceCm <= points.lastKey();
    }

    private static TreeMap<Double, Double> points(double[][] pairs) {
        TreeMap<Double, Double> points = new TreeMap<>();
        if (pairs == null) throw new IllegalArgumentException("Tabela de RPM nula");
        for (double[] pair : pairs) {
            if (pair == null || pair.length != 2) {
                throw new IllegalArgumentException("Cada par precisa de distancia (cm) e RPM");
            }
            if (pair[0] == 0) continue;
            if (!Double.isFinite(pair[0]) || pair[0] < 0
                    || !Double.isFinite(pair[1]) || pair[1] <= 0) {
                throw new IllegalArgumentException("Par distancia/RPM invalido");
            }
            if (points.put(pair[0], pair[1]) != null) {
                throw new IllegalArgumentException("Distancia de calibracao duplicada");
            }
        }
        return points;
    }
}
