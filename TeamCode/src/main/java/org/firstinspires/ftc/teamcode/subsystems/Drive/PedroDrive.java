package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.AprilTagObservation;

import java.util.function.LongSupplier;

/** Drive field-centric do Pedro com hold automatico quando os controles sao soltos. */
public class PedroDrive {
    private static final double PRECISION_MULTIPLIER = 0.3;
    private static final double HOLD_INPUT_THRESHOLD = 0.1;
    private static final double HOLD_VELOCITY_THRESHOLD = 0.2;
    private static final double AIM_TOLERANCE_DEGREES = 1.0;
    private static final double MAX_AIM_CORRECTION_DEGREES = 8.0;
    // Potencia por rad/s: freia pela velocidade medida, sem derivar o tx ruidoso.
    public static final double AIM_TURN_DAMPING = 0.18;
    public static final double AIM_MAX_TURN_POWER = 0.35;
    public static final double AIM_SETTLED_SPEED_RAD_S = Math.toRadians(10);
    private static final int VISION_CONFIRMATION_FRAMES = 3;

    private enum AimSource { NONE, LIMELIGHT }

    private final Follower follower;
    private final Telemetry telemetry;
    private final Controller aimHeadingController;
    private final LongSupplier nanoClock;
    private final HeadingHistory headingHistory = new HeadingHistory();
    private final VisionHeadingTarget teleOpVisionTarget = new VisionHeadingTarget();
    private boolean teleOpVisionActive;
    private boolean visionSettled;
    private double aimTurnCommand;
    private double fieldForwardHeading;
    private AimSource aimSource = AimSource.NONE;
    private double aimHeading;
    private int aimFirstTagId;
    private double lastAimFrameTimestamp = Double.NaN;
    private double lastVisionErrorDegrees = Double.NaN;
    private Pose aimHoldPosition;
    private String aimStatus = "Manual";
    private boolean visionAimActive;
    private boolean visionHoldMode;
    private int visionFirstTagId;
    private int visionValidFrames;
    private double visionLastFrameTimestamp = Double.NaN;
    private double visionHoldHeading;

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
        rememberHeading();
        if (aimHoldPosition != null) {
            holdAim(visionHoldHeading);
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
        aimHoldPosition = null;
        clearVisionAim();
        aimWithObservation(axial, lateral, yaw, precisionMode, observation, false);
    }

    private void aimWithObservation(double axial, double lateral, double yaw, boolean precisionMode,
                                    AprilTagObservation observation, boolean stabilizeTeleOp) {
        if (observation == null || !observation.isVisionReady() || !observation.hasTarget()) {
            run(axial, lateral, yaw, precisionMode);
            aimStatus = observation != null && observation.isVisionReady()
                    ? "Sem alvo: giro manual" : "Sem visao: giro manual";
            return;
        }

        Pose current = follower.pose();
        rememberHeading();
        if (aimSource != AimSource.LIMELIGHT || aimFirstTagId != observation.getFirstTagId()
                || teleOpVisionActive != stabilizeTeleOp) {
            aimHeadingController.reset();
            lastAimFrameTimestamp = Double.NaN;
            teleOpVisionTarget.reset();
        }
        teleOpVisionActive = stabilizeTeleOp;
        if (!stabilizeTeleOp) teleOpVisionTarget.reset();
        aimSource = AimSource.LIMELIGHT;
        aimFirstTagId = observation.getFirstTagId();
        if (observation.getFrameTimestamp() != lastAimFrameTimestamp) {
            double capturedHeading = headingHistory.at(nanoClock.getAsLong()
                    - (long) (observation.getImageAgeSeconds() * 1e9));
            if (!Double.isFinite(capturedHeading)) {
                // No inicio ainda pode nao haver historico suficiente para a imagem.
                capturedHeading = current.heading() - angularSpeed() * observation.getImageAgeSeconds();
            }
            double targetHeading = capturedHeading - Math.toRadians(observation.getTxDegrees());
            lastVisionErrorDegrees = -Math.toDegrees(Angle.normalizeSigned(targetHeading - current.heading()));
            if (stabilizeTeleOp) targetHeading = teleOpVisionTarget.update(targetHeading);
            double error = -Math.toDegrees(Angle.normalizeSigned(targetHeading - current.heading()));
            boolean aligned = Math.abs(error) <= AIM_TOLERANCE_DEGREES;
            double correction = aligned ? 0 : Math.max(-MAX_AIM_CORRECTION_DEGREES,
                    Math.min(MAX_AIM_CORRECTION_DEGREES, error));
            // tx positivo significa alvo a direita; heading positivo e anti-horario.
            aimHeading = Angle.normalizeSigned(current.heading() - Math.toRadians(correction));
            lastAimFrameTimestamp = observation.getFrameTimestamp();
        }
        double alignmentError = teleOpVisionActive
                ? Math.toDegrees(teleOpVisionTarget.error(current.heading())) : lastVisionErrorDegrees;
        boolean aligned = Math.abs(alignmentError) <= AIM_TOLERANCE_DEGREES
                && Math.abs(angularSpeed()) <= AIM_SETTLED_SPEED_RAD_S;
        aimStatus = aligned ? "Alinhado"
                : "Centralizando hive " + aimFirstTagId + "-" + (aimFirstTagId + 3);

        runAim(axial, lateral, precisionMode);
    }

    /** RB do TeleOp: usa so a CELL vista; sem visao confirmada, o giro e manual. */
    public void aimVision(double axial, double lateral, double yaw, boolean precisionMode,
                          AprilTagObservation observation) {
        aimHoldPosition = null;
        runVisionAim(axial, lateral, yaw, precisionMode, observation);
    }

    /** Limelight define o giro; Pedro conserva a posicao de disparo do path. */
    public void aimVisionHold(Pose position, AprilTagObservation observation) {
        aimHoldPosition = position;
        runVisionAim(0, 0, 0, false, observation);
    }

    private void runVisionAim(double axial, double lateral, double yaw, boolean precisionMode,
                              AprilTagObservation observation) {
        rememberHeading();
        int firstTagId = observation == null ? -1 : observation.getFirstTagId();
        boolean holdMode = aimHoldPosition != null;
        if (!visionAimActive || visionFirstTagId != firstTagId || visionHoldMode != holdMode) {
            clearHeadingAim();
            clearVisionAim();
            visionAimActive = true;
            visionFirstTagId = firstTagId;
            visionHoldMode = holdMode;
            visionHoldHeading = holdMode ? aimHoldPosition.heading() : follower.pose().heading();
        }

        boolean valid = observation != null && observation.isVisionReady() && observation.hasTarget();
        if (valid) {
            double timestamp = observation.getFrameTimestamp();
            if (timestamp < visionLastFrameTimestamp) {
                clearHeadingAim();
                visionValidFrames = 0;
            }
            if (timestamp != visionLastFrameTimestamp) {
                visionValidFrames = Math.min(VISION_CONFIRMATION_FRAMES, visionValidFrames + 1);
                visionLastFrameTimestamp = timestamp;
            }
            if (visionValidFrames >= VISION_CONFIRMATION_FRAMES) {
                aimWithObservation(axial, lateral, yaw, precisionMode, observation, !holdMode);
                visionHoldHeading = aimHeading;
                aimStatus = "Limelight: " + aimStatus;
                return;
            }
        } else {
            visionValidFrames = 0;
            visionLastFrameTimestamp = Double.NaN;
        }

        // Nenhuma coordenada da HIVE participa da mira. Sem alvo, nao inicia uma busca.
        clearHeadingAim();
        runManualDrive(axial, lateral, yaw, precisionMode);
        aimStatus = (valid ? "Confirmando Limelight " + visionValidFrames + "/" + VISION_CONFIRMATION_FRAMES
                : "Sem CELL visivel") + (holdMode ? ": mantendo hold" : ": giro manual");
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
        double turn = powers.turn() - AIM_TURN_DAMPING * angularSpeed();
        visionSettled = teleOpVisionActive
                && teleOpVisionTarget.settled(follower.pose().heading(), angularSpeed());
        if (visionSettled) {
            // Na faixa central, ruido do tx e do giroscopio nao deve virar giro alternado.
            // Giro real ainda rapido recebe apenas frenagem, sem perseguir uma nova referencia.
            turn = Math.abs(angularSpeed()) <= AIM_SETTLED_SPEED_RAD_S
                    ? 0 : -AIM_TURN_DAMPING * angularSpeed();
        }
        aimTurnCommand = Math.max(-AIM_MAX_TURN_POWER, Math.min(AIM_MAX_TURN_POWER, turn)) * multiplier;
        // Nao usa driveOrHold aqui: o hold de pose prenderia X/Y e poderia ignorar giros pequenos.
        follower.manual(new DrivePowers(powers.forward(), powers.strafe(),
                aimTurnCommand));
        follower.update();
    }

    private void rememberHeading() {
        headingHistory.record(nanoClock.getAsLong(), follower.pose().heading());
    }

    private double angularSpeed() {
        double speed = follower.velocity().omega;
        return Double.isFinite(speed) ? speed : 0;
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

    public boolean isVisionAligned() {
        double alignmentError = teleOpVisionActive
                ? Math.toDegrees(teleOpVisionTarget.error(follower.pose().heading())) : lastVisionErrorDegrees;
        return visionAimActive && visionValidFrames >= VISION_CONFIRMATION_FRAMES
                && aimSource == AimSource.LIMELIGHT
                && Double.isFinite(follower.velocity().omega)
                && Math.abs(angularSpeed()) <= AIM_SETTLED_SPEED_RAD_S
                && Math.abs(alignmentError) <= AIM_TOLERANCE_DEGREES
                && Math.abs(Angle.normalizeSigned(aimHeading - follower.pose().heading()))
                <= Math.toRadians(AIM_TOLERANCE_DEGREES);
    }

    private void clearAim() {
        aimHoldPosition = null;
        clearHeadingAim();
        clearVisionAim();
    }

    private void clearHeadingAim() {
        teleOpVisionActive = visionSettled = false;
        aimTurnCommand = 0;
        teleOpVisionTarget.reset();
        if (aimSource != AimSource.NONE) {
            aimHeadingController.reset();
        }
        aimSource = AimSource.NONE;
        lastAimFrameTimestamp = Double.NaN;
        lastVisionErrorDegrees = Double.NaN;
    }

    private void clearVisionAim() {
        visionAimActive = false;
        visionValidFrames = 0;
        visionLastFrameTimestamp = Double.NaN;
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
        if (aimSource != AimSource.NONE) {
            if (aimHoldPosition == null) telemetry.addData("Mira giro comandado", aimTurnCommand);
            if (teleOpVisionActive) {
                telemetry.addData("Mira erro visual bruto (graus)", lastVisionErrorDegrees);
                telemetry.addData("Mira erro visual filtrado (graus)",
                        Math.toDegrees(teleOpVisionTarget.error(follower.pose().heading())));
                telemetry.addData("Mira na faixa estavel", visionSettled);
            }
            telemetry.addData("Mira giro medido (graus/s)", Math.toDegrees(angularSpeed()));
            telemetry.addData("Mira erro atual (graus)",
                    Math.toDegrees(Angle.normalizeSigned(aimHeading - follower.pose().heading())));
        }
        telemetry.addData("Robot X (in)", follower.pose().x());
        telemetry.addData("Robot Y (in)", follower.pose().y());
        telemetry.addData("Field Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Heading dos controles (deg)", Math.toDegrees(fieldCentricHeading()));
    }

    public void stop() {
        clearAim();
        headingHistory.clear();
        aimStatus = "Manual";
        follower.stop();
        follower.drivetrain.stop();
    }
}
