package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
import java.util.Arrays;
import java.util.OptionalDouble;
import java.util.concurrent.TimeUnit;
import java.util.function.LongSupplier;

/** Publica heading e distancia 3D do centro da CELL, com validacao de frames. */
public class Limelight {
    public static final String DEVICE_NAME = "limelight";
    public static final int PIPELINE_INDEX = 8;
    public static final boolean UPSIDE_DOWN = true;
    public static final int POLL_RATE_HZ = 100;
    public static final long MAX_FRAME_AGE_MS = 150;
    public static final long DISTANCE_WINDOW_MS = 1000;
    public static final int DISTANCE_MAX_SAMPLES = 40;
    // BIOBUZZ, manual secao 9.9 / figuras 9-15 a 9-17.
    public static final int RED_AUDIENCE_FIRST_TAG = 34;
    public static final int RED_OPPOSITE_FIRST_TAG = 30;
    public static final int BLUE_AUDIENCE_FIRST_TAG = 38;
    public static final int BLUE_OPPOSITE_FIRST_TAG = 42;
    public static final int TAGS_PER_HIVE = 4;

    // Deslocamento local ate o centro do cluster da CELL (nao a abertura).
    // IDs crescentes da esquerda para a direita olhando de frente o adesivo:
    // centros a -6.5, -2.75, +2.75, +6.5 pol. Nao espelhar por alianca;
    // a pose 3D ja considera a orientacao da CELL, inclusive sua inclinacao.
    private static final double[] TAG_TO_CENTER_METERS = {0.1651, 0.06985, -0.06985, -0.1651};
    private static final long WINDOW_NANOS = TimeUnit.MILLISECONDS.toNanos(DISTANCE_WINDOW_MS);
    private static final long SAMPLE_INTERVAL_NANOS = WINDOW_NANOS / DISTANCE_MAX_SAMPLES;

    private final Limelight3A camera;
    private final Telemetry telemetry;
    private final boolean upsideDown;
    private final int pipelineIndex;
    private final LongSupplier nanoClock;
    // Janela circular da distancia ao centro; so muda com novas amostras.
    private final double[] distanceSamples;
    private final long[] distanceSampleTimes;
    private int distanceHead;
    private int distanceCount;
    private int distanceHive;
    private double lastDistanceTimestamp = Double.NaN;
    private long lastDistanceSampleNanos;
    private boolean redAlliance;
    private boolean audienceCell;
    private int selectedFirstTagId = BLUE_OPPOSITE_FIRST_TAG;
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
        this(hardwareMap.tryGet(Limelight3A.class, DEVICE_NAME), telemetry,
                UPSIDE_DOWN, PIPELINE_INDEX, System::nanoTime);
    }

    Limelight(Limelight3A camera, Telemetry telemetry, boolean upsideDown,
              int pipelineIndex, LongSupplier nanoClock) {
        if (DISTANCE_MAX_SAMPLES <= 0 || WINDOW_NANOS < DISTANCE_MAX_SAMPLES) {
            throw new IllegalArgumentException("Janela de distancia invalida");
        }
        distanceSamples = new double[DISTANCE_MAX_SAMPLES];
        distanceSampleTimes = new long[DISTANCE_MAX_SAMPLES];
        this.camera = camera;
        this.telemetry = telemetry;
        this.upsideDown = upsideDown;
        this.pipelineIndex = pipelineIndex;
        this.nanoClock = nanoClock;
        if (camera != null) {
            camera.setPollRateHz(POLL_RATE_HZ);
            camera.pipelineSwitch(pipelineIndex);
            camera.start();
        }
    }

    /** Esquerda/dir: CELL plateia/oposta; cima/baixo: azul/vermelho. */
    public void run(boolean dpadLeft, boolean dpadRight, boolean selectBlue, boolean selectRed) {
        // Comandos opostos simultaneos preservam a selecao daquele eixo.
        if (selectBlue != selectRed) redAlliance = selectRed;
        if (dpadLeft != dpadRight) audienceCell = dpadLeft;
        selectedFirstTagId = redAlliance
                ? (audienceCell ? RED_AUDIENCE_FIRST_TAG : RED_OPPOSITE_FIRST_TAG)
                : (audienceCell ? BLUE_AUDIENCE_FIRST_TAG : BLUE_OPPOSITE_FIRST_TAG);
        long now = nanoClock.getAsLong();
        readFrame(now);
        filterDistance(now);
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
                || age > MAX_FRAME_AGE_MS) {
            clearFrame("Imagem antiga ou invalida");
            return;
        }
        if (timestamp != lastFrameTimestamp) {
            lastFrameTimestamp = timestamp;
            lastNewFrameNanos = now;
        }
        if (now - lastNewFrameNanos > TimeUnit.MILLISECONDS.toNanos(MAX_FRAME_AGE_MS)) {
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
        CellCenter center = estimateCenter(tags);
        if (center != null) {
            double tx = Math.toDegrees(Math.atan2(center.x, center.z));
            observation = AprilTagObservation.detected(selectedFirstTagId,
                    upsideDown ? -tx : tx, center.tagsUsed, timestamp);
            rawDistanceMeters = OptionalDouble.of(center.distanceMeters);
            status = "Centro do hive estimado";
            distanceStatus = "Pose 3D (camera -> centro da CELL)";
        } else if (tags != null) {
            for (FiducialResult tag : tags) {
                if (tag != null && containsTag(selectedFirstTagId, tag.getFiducialId())) {
                    observation = AprilTagObservation.unavailable(selectedFirstTagId);
                    status = "Tag sem pose 3D valida: confira Full3D";
                    distanceStatus = status;
                    break;
                }
            }
        }
    }

    private CellCenter estimateCenter(List<FiducialResult> tags) {
        int firstTagId = selectedFirstTagId;
        if (tags == null || !isHive(firstTagId)) {
            return null;
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

        if (tagsUsed == 0) return null;
        double x = sumX / totalWeight;
        double y = sumY / totalWeight;
        double z = sumZ / totalWeight;
        double distance = Math.hypot(Math.hypot(x, y), z);
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)
                || !Double.isFinite(distance) || distance <= 0) return null;
        return new CellCenter(x, z, distance, tagsUsed);
    }

    /** A mira e a distancia usam a mesma estimativa do centro, inclusive sem tdist. */
    private static final class CellCenter {
        final double x, z, distanceMeters;
        final int tagsUsed;

        CellCenter(double x, double z, double distanceMeters, int tagsUsed) {
            this.x = x;
            this.z = z;
            this.distanceMeters = distanceMeters;
            this.tagsUsed = tagsUsed;
        }
    }

    private void filterDistance(long now) {
        if (!rawDistanceMeters.isPresent() || !Double.isFinite(lastFrameTimestamp)
                || lastFrameTimestamp < 0) {
            clearDistanceFilter();
            return;
        }
        if (distanceHive != selectedFirstTagId || lastFrameTimestamp < lastDistanceTimestamp) {
            clearDistanceFilter();
            distanceHive = selectedFirstTagId;
        }
        boolean changed = false;
        while (distanceCount > 0 && now - distanceSampleTimes[distanceHead] >= WINDOW_NANOS) {
            distanceHead = (distanceHead + 1) % DISTANCE_MAX_SAMPLES;
            distanceCount--;
            changed = true;
        }
        if (lastFrameTimestamp != lastDistanceTimestamp) {
            lastDistanceTimestamp = lastFrameTimestamp;
            if (distanceCount == 0 || now - lastDistanceSampleNanos >= SAMPLE_INTERVAL_NANOS) {
                if (distanceCount == DISTANCE_MAX_SAMPLES) {
                    distanceHead = (distanceHead + 1) % DISTANCE_MAX_SAMPLES;
                    distanceCount--;
                }
                int tail = (distanceHead + distanceCount) % DISTANCE_MAX_SAMPLES;
                distanceSamples[tail] = rawDistanceMeters.getAsDouble();
                distanceSampleTimes[tail] = now;
                distanceCount++;
                lastDistanceSampleNanos = now;
                changed = true;
            }
        }
        if (changed) distanceMeters = trimmedMean();
    }

    private OptionalDouble trimmedMean() {
        if (distanceCount == 0) return OptionalDouble.empty();
        double[] sorted = new double[distanceCount];
        for (int i = 0; i < distanceCount; i++) {
            sorted[i] = distanceSamples[(distanceHead + i) % DISTANCE_MAX_SAMPLES];
        }
        Arrays.sort(sorted);
        int trim = sorted.length / 4;
        int count = sorted.length - 2 * trim;
        double mean = 0;
        for (int i = trim; i < sorted.length - trim; i++) mean += sorted[i] / count;
        return OptionalDouble.of(mean);
    }

    private void clearDistanceFilter() {
        distanceHead = distanceCount = 0;
        distanceMeters = OptionalDouble.empty();
        lastDistanceTimestamp = Double.NaN;
        lastDistanceSampleNanos = 0;
    }

    private static boolean isHive(int firstTagId) {
        return firstTagId == RED_AUDIENCE_FIRST_TAG || firstTagId == RED_OPPOSITE_FIRST_TAG
                || firstTagId == BLUE_AUDIENCE_FIRST_TAG || firstTagId == BLUE_OPPOSITE_FIRST_TAG;
    }

    private static boolean containsTag(int firstTagId, int tagId) {
        return isHive(firstTagId) && tagId >= firstTagId && tagId < firstTagId + TAGS_PER_HIVE;
    }

    private void clearFrame(String reason) {
        observation = AprilTagObservation.unavailable(selectedFirstTagId);
        rawDistanceMeters = OptionalDouble.empty();
        decodedTimestamp = Double.NaN;
        status = reason;
        distanceStatus = reason;
    }

    public boolean isRedAlliance() { return redAlliance; }
    public AprilTagObservation getObservation() { return observation; }
    /** Distancia 3D filtrada da camera ao centro do cluster da CELL, em metros. */
    public OptionalDouble getDistanceMeters() { return distanceMeters; }
    public OptionalDouble getRawDistanceMeters() { return rawDistanceMeters; }
    public int getDistanceSampleCount() { return distanceCount; }
    public String getDistanceStatus() { return distanceStatus; }

    public void sendTelemetry() {
        if (telemetry == null) return;
        telemetry.addData("Limelight", status);
        telemetry.addData("Limelight Pipeline", pipelineIndex);
        telemetry.addData("Alianca", redAlliance ? "VERMELHA" : "AZUL");
        telemetry.addData("CELL selecionada", audienceCell ? "Lado da plateia" : "Oposta a plateia");
        telemetry.addData("Hive selecionado", "%d-%d", selectedFirstTagId,
                selectedFirstTagId + TAGS_PER_HIVE - 1);
        telemetry.addData("Tags usadas no centro", observation.getTagsUsed());
        if (observation.hasTarget()) {
            telemetry.addData("Centro hive erro horizontal (deg)", observation.getTxDegrees());
        } else {
            telemetry.addData("Centro hive erro horizontal (deg)", "Indisponivel");
        }
        telemetry.addData("Distancia centro status", distanceStatus);
        telemetry.addData("Distancia centro amostras", distanceCount);
        if (rawDistanceMeters.isPresent()) {
            telemetry.addData("Distancia centro instantanea (cm)", "%.3f", rawDistanceMeters.getAsDouble() * 100);
        } else {
            telemetry.addData("Distancia centro instantanea (cm)", "Indisponivel");
        }
        if (distanceMeters.isPresent()) {
            telemetry.addData("Distancia centro filtrada (cm)", "%.3f", distanceMeters.getAsDouble() * 100);
        } else {
            telemetry.addData("Distancia centro filtrada (cm)", "Indisponivel");
        }
    }

    public void stop() {
        clearFrame("Parada");
        clearDistanceFilter();
        distanceMeters = OptionalDouble.empty();
        lastFrameTimestamp = Double.NaN;
        if (camera != null) camera.stop();
    }
}
