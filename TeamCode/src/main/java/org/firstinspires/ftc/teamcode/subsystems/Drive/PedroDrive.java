package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.AprilTagObservation;

import java.util.concurrent.TimeUnit;
import java.util.function.LongSupplier;

/** Drive field-centric do Pedro com hold automatico quando os controles sao soltos. */
public class PedroDrive {
    private static final double PRECISION_MULTIPLIER = 0.3;
    private static final double HOLD_INPUT_THRESHOLD = 0.1;
    private static final double HOLD_VELOCITY_THRESHOLD = 0.2;
    private static final double AIM_TOLERANCE_DEGREES = 1.0;
    private static final double MAX_AIM_CORRECTION_DEGREES = 8.0;
    private static final double MIN_AIM_DISTANCE_INCHES = 1e-3;
    private static final int HYBRID_VISION_CONFIRMATION_FRAMES = 3;
    private static final long HYBRID_VISION_LOSS_GRACE_NANOS = TimeUnit.MILLISECONDS.toNanos(100);

    private enum AimSource { NONE, LIMELIGHT, POSITION }

    private final Follower follower;
    private final Telemetry telemetry;
    private final Controller aimHeadingController;
    private final LongSupplier nanoClock;
    private double fieldForwardHeading;
    private AimSource aimSource = AimSource.NONE;
    private double aimHeading;
    private double aimTargetX;
    private double aimTargetY;
    private int aimFirstTagId;
    private double lastAimFrameTimestamp = Double.NaN;
    private double lastVisionErrorDegrees = Double.NaN;
    private Pose aimHoldPosition;
    private String aimStatus = "Manual";
    private boolean hybridActive;
    private boolean hybridVisionLocked;
    private boolean hybridVisionMissing;
    private int hybridFirstTagId;
    private int hybridValidFrames;
    private double hybridTargetX, hybridTargetY;
    private double hybridLastFrameTimestamp = Double.NaN;
    private long hybridVisionLostAtNanos;

    public PedroDrive(Follower follower, Telemetry telemetry, Controller aimHeadingController) {
        this(follower, telemetry, aimHeadingController, System::nanoTime);
    }

    PedroDrive(Follower follower, Telemetry telemetry, Controller aimHeadingController,
               LongSupplier nanoClock) {
        this.follower = follower;
        this.telemetry = telemetry;
        this.aimHeadingController = aimHeadingController;
        this.nanoClock = nanoClock;
    }

    public void run(double axial, double lateral, double yaw, boolean precisionMode) {
        clearAim();
        aimStatus = "Manual";
        runManualDrive(axial, lateral, yaw, precisionMode);
    }

    private void runManualDrive(double axial, double lateral, double yaw, boolean precisionMode) {
        if (aimHoldPosition != null) {
            holdAim(aimHoldPosition.heading());
            return;
        }
        double multiplier = precisionMode ? PRECISION_MULTIPLIER : 1.0;

        // Pedro usa lateral para a esquerda e giro anti-horario positivos.
        DrivePowers powers = ManualDrive.fieldCentric(
                axial * multiplier,
                -lateral * multiplier,
                -yaw * multiplier,
                fieldCentricHeading());

        // Ajusta o limiar junto com a potencia para preservar os comandos no modo precisao.
        ManualDrive.driveOrHold(follower, powers,
                HOLD_INPUT_THRESHOLD * multiplier, HOLD_VELOCITY_THRESHOLD);
        follower.update();
    }

    /** Transla pelo stick; corrige so o heading com alvo visivel, senao libera o giro. */
    public void aim(double axial, double lateral, double yaw, boolean precisionMode,
                    AprilTagObservation observation) {
        if (!observation.isVisionReady() || !observation.hasTarget()) {
            run(axial, lateral, yaw, precisionMode);
            aimStatus = observation.isVisionReady()
                    ? "Sem alvo: giro manual" : "Sem visao: giro manual";
            return;
        }

        Pose current = follower.pose();
        if (aimSource != AimSource.LIMELIGHT || aimFirstTagId != observation.getFirstTagId()) {
            aimHeadingController.reset();
            lastAimFrameTimestamp = Double.NaN;
        }
        aimSource = AimSource.LIMELIGHT;
        aimFirstTagId = observation.getFirstTagId();
        double error = observation.getTxDegrees();
        lastVisionErrorDegrees = error;
        boolean aligned = Math.abs(error) <= AIM_TOLERANCE_DEGREES;
        if (observation.getFrameTimestamp() != lastAimFrameTimestamp) {
            double correction = aligned ? 0 : Math.max(-MAX_AIM_CORRECTION_DEGREES,
                    Math.min(MAX_AIM_CORRECTION_DEGREES, error));
            // tx positivo significa alvo a direita; heading positivo e anti-horario.
            aimHeading = Angle.normalizeSigned(current.heading() - Math.toRadians(correction));
            lastAimFrameTimestamp = observation.getFrameTimestamp();
        }
        aimStatus = aligned ? "Alinhado"
                : "Centralizando hive " + aimFirstTagId + "-" + (aimFirstTagId + 3);

        runAim(axial, lateral, precisionMode);
    }

    /** Mira em X/Y globais sem visao; recalcula o rumo enquanto o robo se desloca. */
    public void aimAt(double axial, double lateral, double yaw, boolean precisionMode,
                      double targetX, double targetY) {
        Pose current = follower.pose();
        double dx = targetX - current.x();
        double dy = targetY - current.y();
        if (!Double.isFinite(dx) || !Double.isFinite(dy)
                || Math.hypot(dx, dy) < MIN_AIM_DISTANCE_INCHES) {
            clearHeadingAim();
            runManualDrive(axial, lateral, yaw, precisionMode);
            aimStatus = "Coordenada sem direcao valida: giro manual";
            return;
        }

        if (aimSource != AimSource.POSITION || aimTargetX != targetX || aimTargetY != targetY) {
            aimHeadingController.reset();
        }
        aimSource = AimSource.POSITION;
        aimTargetX = targetX;
        aimTargetY = targetY;
        // atan2 usa os mesmos eixos globais do Pedro, independente do reset dos sticks.
        double error = Angle.normalizeSigned(Math.atan2(dy, dx) - current.heading());
        boolean aligned = Math.abs(error) <= Math.toRadians(AIM_TOLERANCE_DEGREES);
        double maxCorrection = Math.toRadians(MAX_AIM_CORRECTION_DEGREES);
        double correction = aligned ? 0 : Math.max(-maxCorrection, Math.min(maxCorrection, error));
        aimHeading = Angle.normalizeSigned(current.heading() + correction);
        aimStatus = aligned ? "Alinhado por coordenadas" : "Centralizando hive por coordenadas";

        runAim(axial, lateral, precisionMode);
    }

    /** Pedro procura a coordenada; visao confirmada assume o ajuste fino da mesma CELL. */
    public void aimHybrid(double axial, double lateral, double yaw, boolean precisionMode,
                          double targetX, double targetY, int firstTagId,
                          AprilTagObservation observation) {
        aimHoldPosition = null;
        runHybridAim(axial, lateral, yaw, precisionMode, targetX, targetY, firstTagId, observation);
    }

    /** Mesma mira hibrida do TeleOp, preservando X/Y em hold para o autonomo. */
    public void aimHybridHold(Pose position, double targetX, double targetY, int firstTagId,
                              AprilTagObservation observation) {
        aimHoldPosition = position;
        runHybridAim(0, 0, 0, false, targetX, targetY, firstTagId, observation);
    }

    private void runHybridAim(double axial, double lateral, double yaw, boolean precisionMode,
                              double targetX, double targetY, int firstTagId,
                              AprilTagObservation observation) {
        if (!hybridActive || hybridFirstTagId != firstTagId
                || Double.compare(hybridTargetX, targetX) != 0
                || Double.compare(hybridTargetY, targetY) != 0) {
            clearHybridAim();
            hybridActive = true;
            hybridFirstTagId = firstTagId;
            hybridTargetX = targetX;
            hybridTargetY = targetY;
        }

        boolean selectedCell = observation != null && observation.getFirstTagId() == firstTagId;
        boolean valid = selectedCell && observation.isVisionReady() && observation.hasTarget();
        if (valid) {
            double timestamp = observation.getFrameTimestamp();
            if (timestamp < hybridLastFrameTimestamp) {
                // Reinicio da camera exige confirmar novas imagens e descartar o heading antigo.
                hybridVisionLocked = false;
                hybridValidFrames = 0;
            }
            if (timestamp != hybridLastFrameTimestamp) {
                hybridValidFrames = Math.min(HYBRID_VISION_CONFIRMATION_FRAMES, hybridValidFrames + 1);
                hybridLastFrameTimestamp = timestamp;
            }
            hybridVisionMissing = false;
            if (hybridVisionLocked || hybridValidFrames >= HYBRID_VISION_CONFIRMATION_FRAMES) {
                hybridVisionLocked = true;
                aim(axial, lateral, yaw, precisionMode, observation);
                aimStatus = "Limelight: " + aimStatus;
                return;
            }
        } else {
            hybridValidFrames = 0;
            if (hybridVisionLocked && selectedCell) {
                long now = nanoClock.getAsLong();
                if (!hybridVisionMissing) {
                    hybridVisionMissing = true;
                    hybridVisionLostAtNanos = now;
                }
                if (now - hybridVisionLostAtNanos < HYBRID_VISION_LOSS_GRACE_NANOS) {
                    // Apenas conserva o ultimo heading global; nao reaproveita um tx antigo.
                    runAim(axial, lateral, precisionMode);
                    aimStatus = "Limelight: mantendo heading durante perda breve";
                    return;
                }
            }
            hybridVisionLocked = false;
            hybridVisionMissing = false;
            hybridLastFrameTimestamp = Double.NaN;
        }

        aimAt(axial, lateral, yaw, precisionMode, targetX, targetY);
        aimStatus = "Pedro: " + aimStatus
                + (valid ? " | confirmando visao " + hybridValidFrames
                + "/" + HYBRID_VISION_CONFIRMATION_FRAMES : " | procurando CELL selecionada");
    }

    private void runAim(double axial, double lateral, boolean precisionMode) {
        if (aimHoldPosition != null) {
            holdAim(aimHeading);
            return;
        }
        double multiplier = precisionMode ? PRECISION_MULTIPLIER : 1.0;
        DrivePowers translation = ManualDrive.fieldCentric(
                axial * multiplier, -lateral * multiplier, 0, fieldCentricHeading());
        DrivePowers powers = ManualDrive.headingLock(
                follower, aimHeadingController, translation, aimHeading);
        // Nao usa driveOrHold aqui: o hold de pose prenderia X/Y e poderia ignorar giros pequenos.
        follower.manual(new DrivePowers(powers.forward(), powers.strafe(),
                Math.max(-1, Math.min(1, powers.turn())) * multiplier));
        follower.update();
    }

    private void holdAim(double heading) {
        Pose target = new Pose(aimHoldPosition.x(), aimHoldPosition.y(), heading);
        Pose held = follower.holding() ? follower.poseAt(0) : null;
        if (held == null || Math.hypot(held.x() - target.x(), held.y() - target.y()) > 1e-9
                || Math.abs(Angle.normalizeSigned(held.heading() - heading)) > 1e-9) {
            follower.hold(target);
        }
        follower.update();
    }

    public boolean isHybridVisionAligned() {
        return hybridVisionLocked && !hybridVisionMissing && aimSource == AimSource.LIMELIGHT
                && Math.abs(lastVisionErrorDegrees) <= AIM_TOLERANCE_DEGREES
                && Math.abs(Angle.normalizeSigned(aimHeading - follower.pose().heading()))
                <= Math.toRadians(AIM_TOLERANCE_DEGREES);
    }

    private void clearAim() {
        aimHoldPosition = null;
        clearHeadingAim();
        clearHybridAim();
    }

    private void clearHeadingAim() {
        if (aimSource != AimSource.NONE) {
            aimHeadingController.reset();
        }
        aimSource = AimSource.NONE;
        lastAimFrameTimestamp = Double.NaN;
        lastVisionErrorDegrees = Double.NaN;
    }

    private void clearHybridAim() {
        hybridActive = false;
        hybridVisionLocked = false;
        hybridVisionMissing = false;
        hybridValidFrames = 0;
        hybridLastFrameTimestamp = Double.NaN;
    }

    /** Muda somente a referencia dos sticks; preserva X/Y e heading globais do Pedro. */
    public void resetFieldCentric() {
        fieldForwardHeading = follower.pose().heading();
    }

    public double fieldCentricHeading() {
        return Angle.normalizeSigned(follower.pose().heading() - fieldForwardHeading);
    }

    /** Relocaliza em uma pose conhecida e restaura a frente global do campo. */
    public void resetPose(Pose pose) {
        // setPose sozinho manteria o hold antigo e tentaria voltar a outra coordenada.
        stop();
        follower.setPose(pose);
        fieldForwardHeading = 0;
    }

    public void sendTelemetry() {
        telemetry.addData("Drive Mode", follower.mode());
        telemetry.addData("Mira", aimStatus);
        telemetry.addData("Robot X (in)", follower.pose().x());
        telemetry.addData("Robot Y (in)", follower.pose().y());
        telemetry.addData("Field Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Heading dos controles (deg)", Math.toDegrees(fieldCentricHeading()));
    }

    public void stop() {
        clearAim();
        aimStatus = "Manual";
        follower.stop();
        follower.drivetrain.stop();
    }
}
