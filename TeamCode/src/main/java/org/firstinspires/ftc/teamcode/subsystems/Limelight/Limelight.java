package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.concurrent.TimeUnit;
import java.util.function.LongSupplier;

/** Estima o centro e a distancia do hive escolhido usando suas AprilTags. */
public class Limelight {
    private static final long MAX_FRAME_AGE_MS = 150;

    private final Limelight3A camera;
    private final Telemetry telemetry;
    private final boolean upsideDown;
    private final int pipelineIndex;
    private final LongSupplier nanoClock;
    private final CenterDistanceFilter distanceFilter = new CenterDistanceFilter();

    private int selectedFirstTagId = 42;
    private AprilTagObservation observation = AprilTagObservation.unavailable(selectedFirstTagId);
    private OptionalDouble filteredDistanceMeters = OptionalDouble.empty();
    private double lastFrameTimestamp = Double.NaN;
    private long lastNewFrameNanos;
    private String status = "Sem imagem";

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry,
                     boolean upsideDown, int pipelineIndex) {
        this(hardwareMap.tryGet(Limelight3A.class, "limelight"), telemetry,
                upsideDown, pipelineIndex);
    }

    public Limelight(Limelight3A camera, Telemetry telemetry,
                     boolean upsideDown, int pipelineIndex) {
        this(camera, telemetry, upsideDown, pipelineIndex, System::nanoTime);
    }

    Limelight(Limelight3A camera, Telemetry telemetry, boolean upsideDown,
              int pipelineIndex, LongSupplier nanoClock) {
        this.camera = camera;
        this.telemetry = telemetry;
        this.upsideDown = upsideDown;
        this.pipelineIndex = pipelineIndex;
        this.nanoClock = nanoClock;
        if (camera != null) {
            camera.setPollRateHz(100);
            camera.pipelineSwitch(pipelineIndex);
            camera.start();
        }
    }

    public void run(boolean dpadLeft, boolean dpadRight) {
        long now = nanoClock.getAsLong();
        readObservation(dpadLeft, dpadRight, now);
        filteredDistanceMeters = distanceFilter.update(observation, now);
    }

    private void readObservation(boolean dpadLeft, boolean dpadRight, long now) {
        if (dpadLeft != dpadRight) {
            selectedFirstTagId = dpadLeft ? 38 : 42;
        }
        observation = AprilTagObservation.unavailable(selectedFirstTagId);

        if (camera == null || !camera.isRunning() || !camera.isConnected()) {
            status = "Desconectada";
            return;
        }
        LLResult result = camera.getLatestResult();
        if (result == null) {
            status = "Sem imagem";
            return;
        }
        if (result.getPipelineIndex() != pipelineIndex) {
            status = "Pipeline incorreta";
            return;
        }
        double timestamp = result.getTimestamp();
        long age = result.getStaleness();
        if (!Double.isFinite(timestamp) || timestamp < 0 || age < 0 || age > MAX_FRAME_AGE_MS) {
            status = "Imagem antiga ou invalida";
            return;
        }

        if (timestamp != lastFrameTimestamp) {
            lastFrameTimestamp = timestamp;
            lastNewFrameNanos = now;
        }
        if (now - lastNewFrameNanos > TimeUnit.MILLISECONDS.toNanos(MAX_FRAME_AGE_MS)) {
            status = "Imagem congelada";
            return;
        }

        observation = AprilTagObservation.searching(selectedFirstTagId, timestamp);
        status = "Hive selecionado nao visivel";
        if (!result.isValid()) {
            return;
        }
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) {
            return;
        }
        Optional<HiveCenterEstimator.Estimate> estimate =
                HiveCenterEstimator.estimate(tags, selectedFirstTagId);
        if (estimate.isPresent()) {
            HiveCenterEstimator.Estimate center = estimate.get();
            observation = AprilTagObservation.detected(selectedFirstTagId,
                    upsideDown ? -center.txDegrees : center.txDegrees,
                    center.distanceMeters, center.tagsUsed, timestamp);
            status = "Centro do hive estimado";
            return;
        }
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag != null && tag.getFiducialId() >= selectedFirstTagId
                    && tag.getFiducialId() <= selectedFirstTagId + 3) {
                observation = AprilTagObservation.unavailable(selectedFirstTagId);
                status = "Tag sem pose 3D valida: confira Full3D";
                return;
            }
        }
    }

    public AprilTagObservation getObservation() {
        return observation;
    }

    /** Media aparada da distancia ao centro no ultimo segundo; vazia sem leitura valida. */
    public OptionalDouble getDistanceMeters() {
        return filteredDistanceMeters;
    }

    public void sendTelemetry() {
        telemetry.addData("Limelight", status);
        telemetry.addData("Limelight Mount", upsideDown ? "Invertida" : "Normal");
        telemetry.addData("Limelight Pipeline", pipelineIndex);
        telemetry.addData("Hive selecionado", selectedFirstTagId == 38
                ? "Esquerdo (38-41)" : "Direito (42-45)");
        telemetry.addData("Tags usadas no centro", observation.getTagsUsed());
        telemetry.addData("Amostras distancia flywheel", distanceFilter.sampleCount());
        if (observation.hasTarget()) {
            telemetry.addData("Centro hive erro horizontal (deg)", observation.getTxDegrees());
            telemetry.addData("Distancia instantanea centro hive (cm)",
                    observation.getDistanceMeters().getAsDouble() * 100);
        } else {
            telemetry.addData("Centro hive erro horizontal (deg)", "Indisponivel");
            telemetry.addData("Distancia instantanea centro hive (cm)", "Indisponivel");
        }
        if (filteredDistanceMeters.isPresent()) {
            telemetry.addData("Distancia ate centro hive (cm)",
                    filteredDistanceMeters.getAsDouble() * 100);
        } else {
            telemetry.addData("Distancia ate centro hive (cm)", "Indisponivel");
        }
    }

    public void stop() {
        observation = AprilTagObservation.unavailable(selectedFirstTagId);
        distanceFilter.clear();
        filteredDistanceMeters = OptionalDouble.empty();
        status = "Parada";
        if (camera != null) {
            camera.stop();
        }
    }
}
