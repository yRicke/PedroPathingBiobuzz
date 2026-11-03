package org.firstinspires.ftc.teamcode.subsystems.Limelight;

/** Geometria da abertura e da camera; nao depende de POI nem do robotspace do painel. */
final class AimGeometry {
    // AprilTagGameDatabase.getBioBuzzCluster(), SDK FTC 12.0.0, em polegadas.
    // Os membros estao em (+/-6.5 ou +/-2.75, +7.1874, -5.622) relativos a abertura.
    private static final double[] TAG_X_INCHES = {-6.5, -2.75, 2.75, 6.5};
    private static final double TAG_Y_INCHES = 7.1874;
    private static final double TAG_Z_INCHES = -5.622;
    private static final double METERS_PER_INCH = 0.0254;

    private final Point cameraPosition;
    private final double pitch;
    private final double yaw;
    private final double roll;

    /**
     * Origem na saida das bolas, eixos horizontais: X direita, Y baixo, Z frente.
     * Posicao da lente em cm: direita, acima e a frente positivos.
     * Angulos em graus: pitch para cima, yaw para direita, roll horario.
     * Pitch e medido em relacao a horizontal, nao em relacao ao cano inclinado.
     */
    AimGeometry(double cameraRightCm, double cameraAboveCm, double cameraForwardCm,
                double cameraPitchUpDegrees, double cameraYawRightDegrees,
                double cameraRollClockwiseDegrees) {
        if (!finite(cameraRightCm, cameraAboveCm, cameraForwardCm,
                cameraPitchUpDegrees, cameraYawRightDegrees, cameraRollClockwiseDegrees)) {
            throw new IllegalArgumentException("Medidas da camera devem ser finitas");
        }
        cameraPosition = new Point(cameraRightCm / 100, -cameraAboveCm / 100,
                cameraForwardCm / 100);
        pitch = Math.toRadians(cameraPitchUpDegrees);
        yaw = Math.toRadians(cameraYawRightDegrees);
        roll = Math.toRadians(cameraRollClockwiseDegrees);
    }

    /** Transforma tag -> abertura no referencial bruto da camera da Limelight. */
    static Point openingFromTag(int tagIndex, double tagX, double tagY, double tagZ,
                                double roll, double pitch, double yaw) {
        // Vetor do centro da tag ate a origem oficial do cluster (abertura).
        Point localOffset = new Point(-TAG_X_INCHES[tagIndex] * METERS_PER_INCH,
                -TAG_Y_INCHES * METERS_PER_INCH, -TAG_Z_INCHES * METERS_PER_INCH);
        Point offset = rotateZ(rotateY(rotateX(localOffset, roll), pitch), yaw);
        return new Point(tagX + offset.x, tagY + offset.y, tagZ + offset.z);
    }

    /** Rotacao camera -> eixos horizontais do lancador; depois translacao da lente. */
    Point fromCamera(Point target) {
        Point rotated = rotateY(rotateX(rotateZ(target, roll), pitch), yaw);
        return new Point(rotated.x + cameraPosition.x,
                rotated.y + cameraPosition.y, rotated.z + cameraPosition.z);
    }

    private static Point rotateX(Point p, double angle) {
        double c = Math.cos(angle), s = Math.sin(angle);
        return new Point(p.x, c * p.y - s * p.z, s * p.y + c * p.z);
    }

    private static Point rotateY(Point p, double angle) {
        double c = Math.cos(angle), s = Math.sin(angle);
        return new Point(c * p.x + s * p.z, p.y, -s * p.x + c * p.z);
    }

    private static Point rotateZ(Point p, double angle) {
        double c = Math.cos(angle), s = Math.sin(angle);
        return new Point(c * p.x - s * p.y, s * p.x + c * p.y, p.z);
    }

    private static boolean finite(double... values) {
        for (double value : values) if (!Double.isFinite(value)) return false;
        return true;
    }

    static final class Point {
        final double x, y, z;

        Point(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        boolean isFinite() {
            return finite(x, y, z);
        }

        double distance() {
            return Math.hypot(Math.hypot(x, y), z);
        }
    }
}
