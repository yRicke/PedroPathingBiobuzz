package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.LongSupplier;

/** Seleciona AprilTags e corrige a leitura para a montagem fisica da camera. */
public class Limelight {
    private static final long MAX_FRAME_AGE_MS = 150;

    private final Limelight3A camera;
    private final Telemetry telemetry;
    private final boolean upsideDown;
    private final int pipelineIndex;
    private final LongSupplier nanoClock;

    private int selectedTagId = 43;
    private AprilTagObservation observation = AprilTagObservation.unavailable(selectedTagId);
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
        if (dpadLeft != dpadRight) {
            selectedTagId = dpadLeft ? 39 : 43;
        }
        observation = AprilTagObservation.unavailable(selectedTagId);

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

        long now = nanoClock.getAsLong();
        if (timestamp != lastFrameTimestamp) {
            lastFrameTimestamp = timestamp;
            lastNewFrameNanos = now;
        }
        if (now - lastNewFrameNanos > TimeUnit.MILLISECONDS.toNanos(MAX_FRAME_AGE_MS)) {
            status = "Imagem congelada";
            return;
        }

        observation = AprilTagObservation.searching(selectedTagId, timestamp);
        status = "Tag selecionada nao visivel";
        if (!result.isValid()) {
            return;
        }
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null) {
            return;
        }
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == selectedTagId) {
                double tx = tag.getTargetXDegreesNoCrosshair();
                if (Double.isFinite(tx)) {
                    observation = AprilTagObservation.detected(selectedTagId,
                            upsideDown ? -tx : tx, timestamp);
                    status = "Tag selecionada visivel";
                    return;
                }
            }
        }
    }

    public AprilTagObservation getObservation() {
        return observation;
    }

    public void sendTelemetry() {
        telemetry.addData("Limelight", status);
        telemetry.addData("Limelight Mount", upsideDown ? "Invertida" : "Normal");
        telemetry.addData("Limelight Pipeline", pipelineIndex);
        telemetry.addData("AprilTag selecionada", selectedTagId);
        if (observation.hasTarget()) {
            telemetry.addData("AprilTag erro horizontal (deg)", observation.getTxDegrees());
        }
    }

    public void stop() {
        observation = AprilTagObservation.unavailable(selectedTagId);
        status = "Parada";
        if (camera != null) {
            camera.stop();
        }
    }
}
