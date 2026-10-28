package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.concurrent.TimeUnit;
import java.util.function.LongSupplier;

/** Publica heading do centro do hive e distancia tdist, com validacao de frames. */
public class Limelight {
    private final Limelight3A camera;
    private final Telemetry telemetry;
    private final boolean upsideDown;
    private final int pipelineIndex;
    private final LongSupplier nanoClock;
    private final DistanceFilter distanceFilter = new DistanceFilter();
    private int selectedFirstTagId = LimelightConfig.RIGHT_HIVE_FIRST_TAG;
    private AprilTagObservation observation = AprilTagObservation.unavailable(selectedFirstTagId);
    private OptionalDouble rawDistanceMeters = OptionalDouble.empty();
    private OptionalDouble distanceMeters = OptionalDouble.empty();
    private String status = "Sem imagem";
    private String distanceStatus = "Sem leitura";
    private double lastFrameTimestamp = Double.NaN;
    private long lastNewFrameNanos;
    private double decodedTimestamp = Double.NaN;
    private int decodedHive;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap.tryGet(Limelight3A.class, LimelightConfig.DEVICE_NAME), telemetry,
                LimelightConfig.UPSIDE_DOWN, LimelightConfig.PIPELINE_INDEX, System::nanoTime);
    }

    Limelight(Limelight3A camera, Telemetry telemetry, boolean upsideDown,
              int pipelineIndex, LongSupplier nanoClock) {
        this.camera = camera;
        this.telemetry = telemetry;
        this.upsideDown = upsideDown;
        this.pipelineIndex = pipelineIndex;
        this.nanoClock = nanoClock;
        if (camera != null) {
            camera.setPollRateHz(LimelightConfig.POLL_RATE_HZ);
            camera.pipelineSwitch(pipelineIndex);
            camera.start();
        }
    }

    public void run(boolean dpadLeft, boolean dpadRight) {
        if (dpadLeft != dpadRight) selectedFirstTagId = dpadLeft
                ? LimelightConfig.LEFT_HIVE_FIRST_TAG : LimelightConfig.RIGHT_HIVE_FIRST_TAG;
        long now = nanoClock.getAsLong();
        readFrame(now);
        distanceMeters = distanceFilter.update(rawDistanceMeters, lastFrameTimestamp,
                selectedFirstTagId, now);
    }

    private void readFrame(long now) {
        if (camera == null || !camera.isRunning() || !camera.isConnected()) {
            clearFrame("Desconectada");
            return;
        }
        LLResult result = camera.getLatestResult();
        if (result == null) {
            clearFrame("Sem imagem");
            return;
        }
        if (result.getPipelineIndex() != pipelineIndex) {
            clearFrame("Pipeline incorreta");
            return;
        }
        double timestamp = result.getTimestamp();
        long age = result.getStaleness();
        if (!Double.isFinite(timestamp) || timestamp < 0 || age < 0
                || age > LimelightConfig.MAX_FRAME_AGE_MS) {
            clearFrame("Imagem antiga ou invalida");
            return;
        }
        if (timestamp != lastFrameTimestamp) {
            lastFrameTimestamp = timestamp;
            lastNewFrameNanos = now;
        }
        if (now - lastNewFrameNanos > TimeUnit.MILLISECONDS.toNanos(LimelightConfig.MAX_FRAME_AGE_MS)) {
            clearFrame("Imagem congelada");
            return;
        }
        if (timestamp != decodedTimestamp || decodedHive != selectedFirstTagId) {
            decodeFrame(result, timestamp);
            decodedTimestamp = timestamp;
            decodedHive = selectedFirstTagId;
        }
    }

    private void decodeFrame(LLResult result, double timestamp) {
        observation = AprilTagObservation.searching(selectedFirstTagId, timestamp);
        rawDistanceMeters = OptionalDouble.empty();
        status = "Hive selecionado nao visivel";
        distanceStatus = "Sem alvo valido";
        if (!result.isValid()) return;
        List<FiducialResult> tags = result.getFiducialResults();
        TdistReader.Reading reading = TdistReader.read(result, tags, selectedFirstTagId);
        rawDistanceMeters = reading.distanceMeters;
        distanceStatus = reading.status;
        Optional<HiveCenterEstimator.Estimate> estimate =
                HiveCenterEstimator.estimate(tags, selectedFirstTagId);
        if (estimate.isPresent()) {
            HiveCenterEstimator.Estimate center = estimate.get();
            observation = AprilTagObservation.detected(selectedFirstTagId,
                    upsideDown ? -center.txDegrees : center.txDegrees, center.tagsUsed, timestamp);
            status = "Centro do hive estimado";
        } else if (tags != null) {
            for (FiducialResult tag : tags) {
                if (tag != null && LimelightConfig.containsTag(selectedFirstTagId, tag.getFiducialId())) {
                    observation = AprilTagObservation.unavailable(selectedFirstTagId);
                    status = "Tag sem pose 3D valida: confira Full3D";
                    break;
                }
            }
        }
    }

    private void clearFrame(String reason) {
        observation = AprilTagObservation.unavailable(selectedFirstTagId);
        rawDistanceMeters = OptionalDouble.empty();
        decodedTimestamp = Double.NaN;
        status = reason;
        distanceStatus = reason;
    }

    public AprilTagObservation getObservation() { return observation; }
    /** tdist filtrado: distancia 3D camera-alvo/POI em metros. */
    public OptionalDouble getDistanceMeters() { return distanceMeters; }
    public OptionalDouble getRawDistanceMeters() { return rawDistanceMeters; }
    public int getDistanceSampleCount() { return distanceFilter.sampleCount(); }
    public String getDistanceStatus() { return distanceStatus; }

    public void sendTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("Limelight", status);
        telemetry.addData("Limelight Pipeline", pipelineIndex);
        telemetry.addData("Hive selecionado", "%d-%d", selectedFirstTagId,
                selectedFirstTagId + LimelightConfig.TAGS_PER_HIVE - 1);
        telemetry.addData("Tags usadas no centro", observation.getTagsUsed());
        if (observation.hasTarget()) {
            telemetry.addData("Centro hive erro horizontal (deg)", observation.getTxDegrees());
        } else {
            telemetry.addData("Centro hive erro horizontal (deg)", "Indisponivel");
        }
        telemetry.addData("tdist status", distanceStatus);
        telemetry.addData("tdist amostras", distanceFilter.sampleCount());
        if (rawDistanceMeters.isPresent()) {
            telemetry.addData("tdist instantaneo (cm)", "%.3f", rawDistanceMeters.getAsDouble() * 100);
        } else {
            telemetry.addData("tdist instantaneo (cm)", "Indisponivel");
        }
        if (distanceMeters.isPresent()) {
            telemetry.addData("tdist filtrado (cm)", "%.3f", distanceMeters.getAsDouble() * 100);
        } else {
            telemetry.addData("tdist filtrado (cm)", "Indisponivel");
        }
    }

    public void stop() {
        clearFrame("Parada");
        distanceFilter.clear();
        distanceMeters = OptionalDouble.empty();
        lastFrameTimestamp = Double.NaN;
        if (camera != null) camera.stop();
    }
}
