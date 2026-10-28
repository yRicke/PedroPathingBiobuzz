package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import org.json.JSONException;
import org.json.JSONObject;

import java.util.List;
import java.util.OptionalDouble;

/** Le o tdist do mesmo frame usado pelo SDK, sem HTTP adicional ou reflexao. */
final class TdistReader {
    private TdistReader() { }

    static Reading read(LLResult result, List<FiducialResult> tags, int firstTagId) {
        if (tags == null || tags.isEmpty()) return Reading.unavailable("Sem AprilTags");
        // tdist pertence ao alvo principal da pipeline, nao ao centro calculado em Java.
        // O SDK nao identifica esse alvo: rejeitamos frames que misturem outros alvos.
        for (FiducialResult tag : tags) {
            if (tag == null || !LimelightConfig.containsTag(firstTagId, tag.getFiducialId())) {
                return Reading.unavailable("Isole as tags do hive selecionado na pipeline");
            }
        }
        try {
            // Confirmado no sources.jar de org.firstinspires.ftc:Hardware:12.0.0:
            // LLResult.toString() devolve jsonData.toString(), incluindo campos novos.
            JSONObject json = new JSONObject(result.toString());
            double distance = json.optDouble("tdist", Double.NaN);
            if (!Double.isFinite(distance) || distance <= 0) {
                return Reading.unavailable("tdist ausente/invalido: confira OS 2026.1+ e Full3D");
            }
            return new Reading(OptionalDouble.of(distance), "tdist (camera -> alvo/POI)");
        } catch (JSONException e) {
            return Reading.unavailable("JSON do LLResult sem tdist legivel");
        }
    }

    static final class Reading {
        final OptionalDouble distanceMeters;
        final String status;

        Reading(OptionalDouble distanceMeters, String status) {
            this.distanceMeters = distanceMeters;
            this.status = status;
        }

        static Reading unavailable(String status) {
            return new Reading(OptionalDouble.empty(), status);
        }
    }
}
