package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.AprilTagObservation;

/** Drive field-centric do Pedro com hold automatico quando os controles sao soltos. */
public class PedroDrive {
    private static final double PRECISION_MULTIPLIER = 0.3;
    private static final double HOLD_INPUT_THRESHOLD = 0.1;
    private static final double HOLD_VELOCITY_THRESHOLD = 0.2;
    private static final double AIM_TOLERANCE_DEGREES = 1.0;
    private static final double MAX_AIM_CORRECTION_DEGREES = 8.0;

    private final Follower follower;
    private final Telemetry telemetry;
    private final Controller aimHeadingController;
    private boolean aiming;
    private double aimHeading;
    private int aimFirstTagId;
    private double lastAimFrameTimestamp = Double.NaN;
    private String aimStatus = "Manual";

    public PedroDrive(Follower follower, Telemetry telemetry, Controller aimHeadingController) {
        this.follower = follower;
        this.telemetry = telemetry;
        this.aimHeadingController = aimHeadingController;
    }

    public void run(double axial, double lateral, double yaw, boolean precisionMode) {
        clearAim();
        aimStatus = "Manual";
        double multiplier = precisionMode ? PRECISION_MULTIPLIER : 1.0;

        // Pedro usa lateral para a esquerda e giro anti-horario positivos.
        DrivePowers powers = ManualDrive.fieldCentric(
                axial * multiplier,
                -lateral * multiplier,
                -yaw * multiplier,
                follower.pose().heading());

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
        if (!aiming || aimFirstTagId != observation.getFirstTagId()) {
            aimHeadingController.reset();
            lastAimFrameTimestamp = Double.NaN;
        }
        aiming = true;
        aimFirstTagId = observation.getFirstTagId();
        double error = observation.getTxDegrees();
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

        double multiplier = precisionMode ? PRECISION_MULTIPLIER : 1.0;
        DrivePowers translation = ManualDrive.fieldCentric(
                axial * multiplier, -lateral * multiplier, 0, current.heading());
        DrivePowers powers = ManualDrive.headingLock(
                follower, aimHeadingController, translation, aimHeading);
        // Nao usa driveOrHold aqui: o hold de pose prenderia X/Y e poderia ignorar giros pequenos.
        follower.manual(new DrivePowers(powers.forward(), powers.strafe(),
                Math.max(-1, Math.min(1, powers.turn())) * multiplier));
        follower.update();
    }

    private void clearAim() {
        if (aiming) {
            aimHeadingController.reset();
        }
        aiming = false;
        lastAimFrameTimestamp = Double.NaN;
    }

    public void sendTelemetry() {
        telemetry.addData("Drive Mode", follower.mode());
        telemetry.addData("Mira", aimStatus);
        telemetry.addData("Robot X (in)", follower.pose().x());
        telemetry.addData("Robot Y (in)", follower.pose().y());
        telemetry.addData("Field Heading (deg)", Math.toDegrees(follower.pose().heading()));
    }

    public void stop() {
        clearAim();
        aimStatus = "Manual";
        follower.stop();
        follower.drivetrain.stop();
    }
}
