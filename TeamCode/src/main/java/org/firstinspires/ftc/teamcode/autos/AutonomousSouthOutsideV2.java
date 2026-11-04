package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.AprilTagObservation;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Trigger.Trigger;

import java.util.concurrent.TimeUnit;
import java.util.function.LongSupplier;

/** Seis trechos, com mira Limelight e RPM por distancia nos disparos apos path1 e path4. */
@Autonomous(name = "AutonomousSouthOutsideV2", group = "Autonomous")
public class AutonomousSouthOutsideV2 extends LinearOpMode {
    private static final long SPIN_UP_MS = 1500;
    private static final long AIM_TIMEOUT_MS = 3000;
    private static final long READY_STABLE_MS = 150;
    private static final long SHOOT_TIME_MS = 3000;
    private static final double RPM_TOLERANCE = 100;

    private enum Phase { WARMING, FOLLOWING, AIMING, SHOOTING, DONE }
    private static final String[] PATH_NAMES = {"path1", "path1_2", "path3", "path4", "path5", "path6"};
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(57.8428, 8.1749, 90);
    private final Pose point1 = poseFactory.of(57.8922, 20.3733, 89.7226);
    private final Pose path1Start = poseFactory.of(57.8922, 18.3733, 90);
    private final Pose path1 = poseFactory.of(31.0559, 8.6016, 180);
    private final Pose point3 = poseFactory.of(8.8197, 8.5043, 180);
    private final Pose point4Start = poseFactory.of(8.8197, 8.5043, 180);
    private final Pose point4 = poseFactory.of(58.9525, 133.3125, 270);
    private final Pose point4Control1 = poseFactory.of(38.1594, 48.3839, 0);
    private final Pose point4Control2 = poseFactory.of(6.5729, 108.8181, 0);
    private final Pose point4Control3 = poseFactory.of(57.3303, 105.8968, 0);
    private final Pose point5 = poseFactory.of(59.1024, 121.7263, -89.2586);
    private final Pose point6Start = poseFactory.of(59.1024, 121.7263, 270);
    private final Pose point6 = poseFactory.of(9.6898, 121.7273, 0);

    private Follower follower;
    private PedroDrive drive;
    private Intake intake;
    private Flywheel flywheel;
    private Transfer transfer;
    private Trigger trigger;
    private Limelight limelight;
    private LongSupplier nanoClock = System::nanoTime;
    private Path[] paths;
    private int pathIndex = -1;
    private Phase phase = Phase.WARMING;
    private boolean audienceCell;
    private boolean redAlliance;
    private long phaseStartedNanos;
    private long readySinceNanos;
    private boolean wasReady;
    private String shotStatus = "Aguardando path1";

    @Override
    public void runOpMode() {
        OpModeStorage.clearAutonomousPose();
        follower = Constants.create(hardwareMap);
        drive = new PedroDrive(follower, telemetry, Constants.foresightConfig.headingFeedback.get());
        intake = new Intake(hardwareMap, telemetry);
        flywheel = new Flywheel(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);
        trigger = new Trigger(hardwareMap, telemetry);
        limelight = new Limelight(hardwareMap, telemetry);
        boolean autonomousStarted = false;

        try {
            intake.offMotor();
            setShooting(false);
            follower.setPose(start);
            do {
                follower.update();
                limelight.run(false, true, gamepad2.dpad_up, gamepad2.dpad_down);
                redAlliance = limelight.isRedAlliance();
                telemetry.addLine("Disparos em hold: fim de path1 e path4, com mira Limelight e RPM por distancia");
                telemetry.addLine("G2 cima: AZUL / baixo: VERMELHA (selecao antes do Play)");
                limelight.sendTelemetry();
                telemetry.update();
                idle();
            } while (opModeInInit());
            waitForStart();
            if (isStopRequested()) return;

            autonomousStarted = true;
            startAutonomous();
            while (opModeIsActive()) {
                updateAutonomous();
                telemetry.addData("Etapa", "%s | %s", phase,
                        pathIndex < 0 ? "inicio" : PATH_NAMES[pathIndex]);
                telemetry.addData("Disparo", shotStatus);
                drive.sendTelemetry();
                flywheel.sendTelemetry();
                limelight.sendTelemetry();
                telemetry.update();
                idle();
            }
        } finally {
            finishAutonomous(autonomousStarted);
        }
    }

    void startAutonomous() {
        paths = new Path[] {path1(), path1_2(), path3(), path4(), path5(), path6()};
        follower.holdEnd.set(true);
        follower.hold(start);
        pathIndex = -1;
        audienceCell = false;
        phase = Phase.WARMING;
        phaseStartedNanos = nanoClock.getAsLong();
        wasReady = false;
    }

    void updateAutonomous() {
        long now = nanoClock.getAsLong();
        limelight.run(audienceCell, !audienceCell, !redAlliance, redAlliance);
        boolean feeding = false;
        if (phase != Phase.DONE) {
            flywheel.runByDistance(limelight.getDistanceMeters());
            intake.onMotor();
        }

        switch (phase) {
            case WARMING:
                follower.update();
                if (elapsedMillis(now, phaseStartedNanos) >= SPIN_UP_MS) advancePath();
                break;
            case FOLLOWING:
                follower.update();
                if (!follower.following() && !follower.isBusy()) {
                    if (pathIndex == 0 || pathIndex == 3) {
                        phase = Phase.AIMING;
                        phaseStartedNanos = now;
                        wasReady = false;
                        shotStatus = "Alinhando e estabilizando RPM apos " + PATH_NAMES[pathIndex];
                    } else {
                        advancePath();
                    }
                }
                break;
            case AIMING:
            case SHOOTING:
                AprilTagObservation observation = limelight.getObservation();
                drive.aimVisionHold(paths[pathIndex].endPose(), observation);
                boolean ready = drive.isVisionAligned() && !follower.isBusy() && rpmReady();
                if (ready && !wasReady) readySinceNanos = now;
                wasReady = ready;
                boolean stableReady = ready && elapsedMillis(now, readySinceNanos) >= READY_STABLE_MS;
                if (phase == Phase.AIMING) {
                    if (elapsedMillis(now, phaseStartedNanos) >= AIM_TIMEOUT_MS) {
                        shotStatus = "Sem alinhamento/RPM: disparo de " + PATH_NAMES[pathIndex] + " ignorado";
                        finishShot();
                    } else if (stableReady) {
                        phase = Phase.SHOOTING;
                        phaseStartedNanos = now;
                        shotStatus = "Disparando apos " + PATH_NAMES[pathIndex];
                        feeding = true;
                    }
                } else if (elapsedMillis(now, phaseStartedNanos) >= SHOOT_TIME_MS) {
                    shotStatus = "Disparo de " + PATH_NAMES[pathIndex] + " concluido";
                    finishShot();
                } else {
                    // A perda de visao, de RPM ou de posicao interrompe a alimentacao nesse ciclo.
                    feeding = stableReady;
                }
                break;
            case DONE:
                follower.update();
                flywheel.stopMotors();
                intake.offMotor();
                break;
        }
        setShooting(feeding);
    }

    private boolean rpmReady() {
        double target = flywheel.getTargetRPM();
        double first = flywheel.getRPM();
        double second = flywheel.getSecondMotorRPM();
        return Double.isFinite(target) && target > 0 && Double.isFinite(first) && Double.isFinite(second)
                && Math.abs(first - target) <= RPM_TOLERANCE
                && Math.abs(second - target) <= RPM_TOLERANCE;
    }

    private void finishShot() {
        setShooting(false);
        drive.stop();
        audienceCell = true;
        advancePath();
    }

    private void advancePath() {
        pathIndex++;
        if (pathIndex < paths.length) {
            follower.follow(paths[pathIndex]);
            phase = Phase.FOLLOWING;
        } else {
            pathIndex = paths.length - 1;
            phase = Phase.DONE;
            flywheel.stopMotors();
            intake.offMotor();
        }
    }

    private static long elapsedMillis(long now, long since) {
        return TimeUnit.NANOSECONDS.toMillis(now - since);
    }

    private void setShooting(boolean enabled) {
        trigger.run(enabled);
        transfer.run(enabled);
    }

    void finishAutonomous(boolean started) {
        try {
            if (started) {
                follower.localizer.update();
                OpModeStorage.saveAutonomousPose(follower.pose());
            }
        } finally {
            try {
                drive.stop();
            } finally {
                try {
                    setShooting(false);
                    intake.offMotor();
                } finally {
                    try {
                        flywheel.stopMotors();
                    } finally {
                        limelight.stop();
                    }
                }
            }
        }
    }

    public Path path1() { return Paths.line(start, point1).tangent(); }
    public Path path1_2() { return Paths.line(path1Start, path1).linear(path1Start, path1); }
    public Path path3() { return Paths.line(path1, point3).constant(point3); }
    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4Control2, point4Control3, point4)
                .linear(point4Start, point4);
    }
    public Path path5() { return Paths.line(point4, point5).tangent(); }
    public Path path6() { return Paths.line(point6Start, point6).linear(point6Start, point6); }
}
