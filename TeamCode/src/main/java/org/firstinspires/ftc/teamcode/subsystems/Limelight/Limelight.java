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

/** Publica mira e distancia 3D da saida das bolas ate a abertura da CELL. */
public class Limelight {
    public static final String DEVICE_NAME = "limelight";
    public static final int PIPELINE_INDEX = 8;
    public static final boolean UPSIDE_DOWN = true;
    // Medidas do CAD e montagem confirmadas em 02/11/2026.
    // Direita, acima e a frente positivos; pitch acima da horizontal e yaw a direita.
    public static final double CAMERA_RIGHT_CM = 0;
    public static final double CAMERA_ABOVE_CM = 12.9;
    public static final double CAMERA_FORWARD_CM = -10.691;
    public static final double CAMERA_PITCH_UP_DEGREES = 49.265;
    public static final double CAMERA_YAW_RIGHT_DEGREES = 0;
    public static final int POLL_RATE_HZ = 100;
    public static final long MAX_FRAME_AGE_MS = 150;
    public static final double MAX_IMAGE_AGE_MS = 300;
    // Desacordo maximo entre estimativas da mesma abertura no mesmo frame.
    public static final double MAX_OPENING_DISAGREEMENT_METERS = 0.12;
    public static final long DISTANCE_WINDOW_MS = 1000;
    public static final int DISTANCE_MAX_SAMPLES = 40;
    public static final long DISTANCE_SMOOTHING_MS = 250;
    public static final double DISTANCE_DEADBAND_METERS = 0.01;
    public static final long DISTANCE_HOLD_MS = 200;
    // BIOBUZZ, manual secao 9.9 / figuras 9-15 a 9-17.
    public static final int RED_AUDIENCE_FIRST_TAG = 34;
    public static final int RED_OPPOSITE_FIRST_TAG = 30;
    public static final int BLUE_AUDIENCE_FIRST_TAG = 38;
    public static final int BLUE_OPPOSITE_FIRST_TAG = 42;
    public static final int TAGS_PER_HIVE = 4;

    private static final long WINDOW_NANOS = TimeUnit.MILLISECONDS.toNanos(DISTANCE_WINDOW_MS);
    private static final long SAMPLE_INTERVAL_NANOS = WINDOW_NANOS / DISTANCE_MAX_SAMPLES;

    private final Limelight3A camera;
    private final Telemetry telemetry;
    private final AimGeometry geometry;
    private final int pipelineIndex;
    private final LongSupplier nanoClock;
    // Janela circular da distancia a abertura; so muda com novas amostras.
    private final double[] distanceSamples;
    private final long[] distanceSampleTimes;
    private int distanceHead;
    private int distanceCount;
    private int distanceHive;
    private double lastDistanceTimestamp = Double.NaN;
    private long lastDistanceSampleNanos;
    private long lastValidDistanceNanos;
    private long lastSmoothedDistanceNanos;
    private double smoothedDistanceMeters;
    private boolean freshFrame;
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
    private int rejectedOpeningPoses;
    private boolean openingWithoutConsensus;

    public Limelight(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap.tryGet(Limelight3A.class, DEVICE_NAME), telemetry,
                new AimGeometry(CAMERA_RIGHT_CM, CAMERA_ABOVE_CM, CAMERA_FORWARD_CM,
                        CAMERA_PITCH_UP_DEGREES, CAMERA_YAW_RIGHT_DEGREES,
                        UPSIDE_DOWN ? 180 : 0), PIPELINE_INDEX, System::nanoTime);
    }

    Limelight(Limelight3A camera, Telemetry telemetry, boolean upsideDown,
              int pipelineIndex, LongSupplier nanoClock) {
        this(camera, telemetry, new AimGeometry(0, 0, 0, 0, 0, upsideDown ? 180 : 0),
                pipelineIndex, nanoClock);
    }

    Limelight(Limelight3A camera, Telemetry telemetry, AimGeometry geometry,
              int pipelineIndex, LongSupplier nanoClock) {
        if (DISTANCE_MAX_SAMPLES <= 0 || WINDOW_NANOS < DISTANCE_MAX_SAMPLES) {
            throw new IllegalArgumentException("Janela de distancia invalida");
        }
        distanceSamples = new double[DISTANCE_MAX_SAMPLES];
        distanceSampleTimes = new long[DISTANCE_MAX_SAMPLES];
        this.camera = camera;
        this.telemetry = telemetry;
        this.geometry = geometry;
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
        freshFrame = false;
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
            double captureMs = result.getCaptureLatency();
            double targetingMs = result.getTargetingLatency();
            double parseMs = result.getParseLatency();
            double imageAgeMs = age + captureMs + targetingMs + parseMs;
            if (!Double.isFinite(imageAgeMs) || captureMs < 0 || targetingMs < 0 || parseMs < 0
                    || imageAgeMs > MAX_IMAGE_AGE_MS) {
                clearFrame("Imagem com latencia invalida ou excessiva");
                return;
            }
            decodeFrame(result, timestamp, imageAgeMs / 1000);
            decodedTimestamp = timestamp;
            decodedHive = selectedFirstTagId;
        }
        freshFrame = true;
    }

    private void decodeFrame(LLResult result, double timestamp, double imageAgeSeconds) {
        rejectedOpeningPoses = 0;
        openingWithoutConsensus = false;
        observation = AprilTagObservation.searching(selectedFirstTagId, timestamp);
        rawDistanceMeters = OptionalDouble.empty();
        status = "Hive selecionado nao visivel";
        distanceStatus = "Sem alvo valido";
        if (!result.isValid()) return;
        List<FiducialResult> tags = result.getFiducialResults();
        Opening opening = estimateOpening(tags);
        if (opening != null) {
            double tx = Math.toDegrees(Math.atan2(opening.position.x, opening.position.z));
            observation = AprilTagObservation.detected(selectedFirstTagId,
                    tx, opening.tagsUsed, timestamp, imageAgeSeconds);
            rawDistanceMeters = OptionalDouble.of(opening.position.distance());
            status = "Abertura da CELL estimada";
            distanceStatus = "Pose 3D (saida das bolas -> abertura da CELL)";
        } else if (openingWithoutConsensus) {
            status = "Poses das tags discordam: abertura sem consenso";
            distanceStatus = status;
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

    private Opening estimateOpening(List<FiducialResult> tags) {
        int firstTagId = selectedFirstTagId;
        if (tags == null || !isHive(firstTagId)) {
            return null;
        }
        AimGeometry.Point[] candidates = new AimGeometry.Point[TAGS_PER_HIVE];
        double[] weights = new double[TAGS_PER_HIVE];

        for (FiducialResult tag : tags) {
            if (tag == null) continue;
            int index = tag.getFiducialId() - firstTagId;
            if (index < 0 || index >= TAGS_PER_HIVE) continue;

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

            AimGeometry.Point opening = AimGeometry.openingFromTag(index,
                    position.x, position.y, position.z, roll, pitch, yaw);
            if (!opening.isFinite() || opening.z <= 0) {
                continue;
            }
            double area = tag.getTargetArea();
            double weight = Double.isFinite(area) && area > 0 ? Math.min(area, 100) : 1;
            if (candidates[index] == null || weight > weights[index]) {
                candidates[index] = opening;
                weights[index] = weight;
            }
        }

        // Maioria de IDs distintos primeiro; uma tag grande com pose ruim nao vence tres boas.
        int count = 0, bestCount = 0, bestIndex = -1;
        double bestWeight = 0;
        for (int i = 0; i < TAGS_PER_HIVE; i++) {
            if (candidates[i] == null) continue;
            count++;
            int agreeing = 0;
            double weight = 0;
            for (int j = 0; j < TAGS_PER_HIVE; j++) {
                if (candidates[j] != null && agree(candidates[i], candidates[j])) {
                    agreeing++;
                    weight += weights[j];
                }
            }
            if (agreeing > bestCount || (agreeing == bestCount && weight > bestWeight)) {
                bestCount = agreeing;
                bestWeight = weight;
                bestIndex = i;
            }
        }
        if (bestIndex < 0) return null;
        if (count > 1 && bestCount <= count / 2) {
            openingWithoutConsensus = true;
            rejectedOpeningPoses = count;
            return null;
        }
        double sumX = 0, sumY = 0, sumZ = 0, totalWeight = 0;
        int tagsUsed = 0;
        for (int i = 0; i < TAGS_PER_HIVE; i++) {
            if (candidates[i] == null || !agree(candidates[bestIndex], candidates[i])) continue;
            sumX += candidates[i].x * weights[i];
            sumY += candidates[i].y * weights[i];
            sumZ += candidates[i].z * weights[i];
            totalWeight += weights[i];
            tagsUsed++;
        }
        rejectedOpeningPoses = count - tagsUsed;
        AimGeometry.Point opening = geometry.fromCamera(new AimGeometry.Point(
                sumX / totalWeight, sumY / totalWeight, sumZ / totalWeight));
        if (!opening.isFinite() || opening.z <= 0
                || !Double.isFinite(opening.distance())) return null;
        return new Opening(opening, tagsUsed);
    }

    private static boolean agree(AimGeometry.Point a, AimGeometry.Point b) {
        return Math.hypot(Math.hypot(a.x - b.x, a.y - b.y), a.z - b.z)
                <= MAX_OPENING_DISAGREEMENT_METERS;
    }

    /** Mira e distancia usam a mesma abertura no referencial da saida das bolas. */
    private static final class Opening {
        final AimGeometry.Point position;
        final int tagsUsed;

        Opening(AimGeometry.Point position, int tagsUsed) {
            this.position = position;
            this.tagsUsed = tagsUsed;
        }
    }

    private void filterDistance(long now) {
        if (distanceHive != selectedFirstTagId || lastFrameTimestamp < lastDistanceTimestamp) {
            clearDistanceFilter();
            distanceHive = selectedFirstTagId;
        }
        if (!rawDistanceMeters.isPresent() || !Double.isFinite(lastFrameTimestamp)
                || lastFrameTimestamp < 0) {
            // Uma falha breve de deteccao nao deve reiniciar a RPM. A mira nao e retida.
            if (freshFrame && distanceMeters.isPresent()
                    && now - lastValidDistanceNanos <= TimeUnit.MILLISECONDS.toNanos(DISTANCE_HOLD_MS)) {
                distanceStatus = "Mantendo distancia: falha breve de deteccao";
                return;
            }
            distanceStatus = status;
            clearDistanceFilter();
            return;
        }
        boolean changed = false;
        while (distanceCount > 0 && now - distanceSampleTimes[distanceHead] >= WINDOW_NANOS) {
            distanceHead = (distanceHead + 1) % DISTANCE_MAX_SAMPLES;
            distanceCount--;
            changed = true;
        }
        if (distanceCount == 0) distanceMeters = OptionalDouble.empty();
        if (lastFrameTimestamp != lastDistanceTimestamp) {
            lastDistanceTimestamp = lastFrameTimestamp;
            lastValidDistanceNanos = now;
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
        if (changed) stabilizeDistance(now);
    }

    private void stabilizeDistance(long now) {
        OptionalDouble mean = trimmedMean();
        if (!mean.isPresent()) {
            distanceMeters = OptionalDouble.empty();
            return;
        }
        if (!distanceMeters.isPresent()) {
            smoothedDistanceMeters = mean.getAsDouble();
            distanceMeters = mean;
        } else {
            // Alpha depende do tempo, mantendo a resposta igual em diferentes taxas de loop.
            double alpha = -Math.expm1(-(double) (now - lastSmoothedDistanceNanos)
                    / TimeUnit.MILLISECONDS.toNanos(DISTANCE_SMOOTHING_MS));
            smoothedDistanceMeters += alpha * (mean.getAsDouble() - smoothedDistanceMeters);
            if (Math.abs(smoothedDistanceMeters - distanceMeters.getAsDouble())
                    >= DISTANCE_DEADBAND_METERS) {
                distanceMeters = OptionalDouble.of(smoothedDistanceMeters);
            }
        }
        lastSmoothedDistanceNanos = now;
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
        lastValidDistanceNanos = lastSmoothedDistanceNanos = 0;
        smoothedDistanceMeters = 0;
    }

    private static boolean isHive(int firstTagId) {
        return firstTagId == RED_AUDIENCE_FIRST_TAG || firstTagId == RED_OPPOSITE_FIRST_TAG
                || firstTagId == BLUE_AUDIENCE_FIRST_TAG || firstTagId == BLUE_OPPOSITE_FIRST_TAG;
    }

    private static boolean containsTag(int firstTagId, int tagId) {
        return isHive(firstTagId) && tagId >= firstTagId && tagId < firstTagId + TAGS_PER_HIVE;
    }

    private void clearFrame(String reason) {
        rejectedOpeningPoses = 0;
        observation = AprilTagObservation.unavailable(selectedFirstTagId);
        rawDistanceMeters = OptionalDouble.empty();
        decodedTimestamp = Double.NaN;
        status = reason;
        distanceStatus = reason;
    }

    public boolean isRedAlliance() { return redAlliance; }
    public AprilTagObservation getObservation() { return observation; }
    /** Distancia 3D filtrada da saida das bolas ate a abertura da CELL, em metros. */
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
        telemetry.addData("Tags usadas na abertura", observation.getTagsUsed());
        telemetry.addData("Poses de tags rejeitadas", rejectedOpeningPoses);
        if (observation.hasTarget()) {
            telemetry.addData("Idade da imagem na leitura (ms)", observation.getImageAgeSeconds() * 1000);
            telemetry.addData("Abertura erro horizontal (deg)", observation.getTxDegrees());
        } else {
            telemetry.addData("Abertura erro horizontal (deg)", "Indisponivel");
        }
        telemetry.addData("Distancia abertura status", distanceStatus);
        telemetry.addData("Distancia abertura amostras", distanceCount);
        if (rawDistanceMeters.isPresent()) {
            telemetry.addData("Distancia abertura instantanea (cm)", "%.3f", rawDistanceMeters.getAsDouble() * 100);
        } else {
            telemetry.addData("Distancia abertura instantanea (cm)", "Indisponivel");
        }
        if (distanceMeters.isPresent()) {
            telemetry.addData("Distancia abertura filtrada (cm)", "%.3f", distanceMeters.getAsDouble() * 100);
        } else {
            telemetry.addData("Distancia abertura filtrada (cm)", "Indisponivel");
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
