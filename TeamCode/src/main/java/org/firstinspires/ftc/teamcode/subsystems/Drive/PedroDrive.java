package org.firstinspires.ftc.teamcode.subsystems.Drive;

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
    private static final double SEARCH_HEADING_STEP_DEGREES = 2.0;

    private final Follower follower;
    private final Telemetry telemetry;
    private Pose aimPose;
    private int aimTagId;
    private double lastAimFrameTimestamp = Double.NaN;
    private double searchDirection = -1;
    private boolean visionWasReady;
    private String aimStatus = "Manual";

    public PedroDrive(Follower follower, Telemetry telemetry) {
        this.follower = follower;
        this.telemetry = telemetry;
    }

    public void run(double axial, double lateral, double yaw, boolean precisionMode) {
        if (aimPose != null) {
            // Soltar o gatilho cancela o alvo de giro, inclusive com sticks neutros.
            follower.hold(follower.pose());
            aimPose = null;
        }
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

    /** Mantem X/Y capturados e usa a camera somente para ajustar o heading. */
    public void aim(AprilTagObservation observation) {
        Pose current = follower.pose();
        if (aimPose == null) {
            aimPose = current;
            aimTagId = observation.getTagId();
            lastAimFrameTimestamp = Double.NaN;
            searchDirection = -1;
            visionWasReady = false;
        }
        if (aimTagId != observation.getTagId()) {
            aimTagId = observation.getTagId();
            lastAimFrameTimestamp = Double.NaN;
            searchDirection = -1;
        }

        if (!observation.isVisionReady()) {
            if (visionWasReady) {
                aimPose = aimPose.withHeading(current.heading());
            }
            aimStatus = "Sem visao: mantendo pose";
        } else if (!observation.hasTarget()) {
            double heading = current.heading()
                    + searchDirection * Math.toRadians(SEARCH_HEADING_STEP_DEGREES);
            aimPose = aimPose.withHeading(Angle.normalizeSigned(heading));
            aimStatus = "Buscando AprilTag " + aimTagId;
        } else {
            double error = observation.getTxDegrees();
            boolean aligned = Math.abs(error) <= AIM_TOLERANCE_DEGREES;
            if (observation.getFrameTimestamp() != lastAimFrameTimestamp) {
                double correction = aligned ? 0 : Math.max(-MAX_AIM_CORRECTION_DEGREES,
                        Math.min(MAX_AIM_CORRECTION_DEGREES, error));
                double heading = current.heading() - Math.toRadians(correction);
                aimPose = aimPose.withHeading(Angle.normalizeSigned(heading));
                lastAimFrameTimestamp = observation.getFrameTimestamp();
                if (!aligned) {
                    // tx positivo significa alvo a direita; heading positivo e anti-horario.
                    searchDirection = -Math.signum(error);
                }
            }
            aimStatus = aligned ? "Alinhado" : "Centralizando AprilTag " + aimTagId;
        }

        visionWasReady = observation.isVisionReady();
        follower.hold(aimPose);
        follower.update();
    }

    public void sendTelemetry() {
        telemetry.addData("Drive Mode", follower.mode());
        telemetry.addData("Mira", aimStatus);
        telemetry.addData("Robot X (in)", follower.pose().x());
        telemetry.addData("Robot Y (in)", follower.pose().y());
        telemetry.addData("Field Heading (deg)", Math.toDegrees(follower.pose().heading()));
    }

    public void stop() {
        aimPose = null;
        aimStatus = "Manual";
        follower.stop();
        follower.drivetrain.stop();
    }
}
