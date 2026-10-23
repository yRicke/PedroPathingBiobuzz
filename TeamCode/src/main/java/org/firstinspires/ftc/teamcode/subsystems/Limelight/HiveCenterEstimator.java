package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
import java.util.Optional;

/** Projeta o centro do hive no espaco da camera a partir das tags visiveis. */
final class HiveCenterEstimator {
    // BIOBUZZ, figura 9-15: centros das tags a -6.5, -2.75, +2.75 e +6.5 in.
    // Deslocamento de cada tag ATE o centro, no eixo X local da tag (metros).
    private static final double[] TAG_TO_CENTER_METERS = {0.1651, 0.06985, -0.06985, -0.1651};

    private HiveCenterEstimator() { }

    static Optional<Estimate> estimate(List<FiducialResult> tags, int firstTagId) {
        if (tags == null || (firstTagId != 38 && firstTagId != 42)) {
            return Optional.empty();
        }
        double sumX = 0;
        double sumY = 0;
        double sumZ = 0;
        double totalWeight = 0;
        int tagsUsed = 0;

        for (FiducialResult tag : tags) {
            if (tag == null) continue;
            int index = tag.getFiducialId() - firstTagId;
            if (index < 0 || index >= TAG_TO_CENTER_METERS.length) continue;

            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose == null || pose.getPosition() == null || pose.getOrientation() == null
                    || pose.getPosition().unit == null) continue;
            Position position = pose.getPosition().toUnit(DistanceUnit.METER);
            YawPitchRollAngles orientation = pose.getOrientation();
            double yaw = orientation.getYaw(AngleUnit.RADIANS);
            double pitch = orientation.getPitch(AngleUnit.RADIANS);
            double roll = orientation.getRoll(AngleUnit.RADIANS);
            if (!Double.isFinite(position.x) || !Double.isFinite(position.y)
                    || !Double.isFinite(position.z) || position.z <= 0
                    || !Double.isFinite(yaw) || !Double.isFinite(pitch)
                    || !Double.isFinite(roll)) continue;

            // Limelight: X direita, Y baixo, Z frente. Rz(yaw)*Ry(pitch)*Rx(roll)
            // transforma o deslocamento local (d, 0, 0) para o espaco da camera.
            double offset = TAG_TO_CENTER_METERS[index];
            double x = position.x + offset * Math.cos(pitch) * Math.cos(yaw);
            double y = position.y + offset * Math.cos(pitch) * Math.sin(yaw);
            double z = position.z - offset * Math.sin(pitch);
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z) || z <= 0) {
                continue;
            }
            double area = tag.getTargetArea();
            double weight = Double.isFinite(area) && area > 0 ? Math.min(area, 100) : 1;
            sumX += x * weight;
            sumY += y * weight;
            sumZ += z * weight;
            totalWeight += weight;
            tagsUsed++;
        }

        if (tagsUsed == 0) return Optional.empty();
        double x = sumX / totalWeight;
        double y = sumY / totalWeight;
        double z = sumZ / totalWeight;
        double distance = Math.hypot(Math.hypot(x, y), z);
        double tx = Math.toDegrees(Math.atan2(x, z));
        if (!Double.isFinite(distance) || distance <= 0 || !Double.isFinite(tx)) {
            return Optional.empty();
        }
        return Optional.of(new Estimate(tx, distance, tagsUsed));
    }

    static final class Estimate {
        final double txDegrees;
        final double distanceMeters;
        final int tagsUsed;

        Estimate(double txDegrees, double distanceMeters, int tagsUsed) {
            this.txDegrees = txDegrees;
            this.distanceMeters = distanceMeters;
            this.tagsUsed = tagsUsed;
        }
    }
}
