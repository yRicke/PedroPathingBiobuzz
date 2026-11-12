package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.autos.AutonomousConstants.*;

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
import org.firstinspires.ftc.teamcode.subsystems.FlowerCollect.FlowerCollect;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Trigger.Trigger;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import java.util.function.LongSupplier;

/** Tres lancamentos em hold e coleta da flower no percurso de paths/AutoSecundario.pp.json. */
@Autonomous(name = "Auto_Secundario", group = "Autonomous")
public class AutoSecundario extends LinearOpMode {
    private static final int SECOND_SHOT_PATH = 3;
    private static final int PREP_FLOWER_PATH = 4;
    private static final int COLLECT_FLOWER_PATH = 5;
    private static final int POST_FLOWER_PATH = 6;
    private static final int THIRD_SHOT_PATH = 7;
    private static final String[] PATH_NAMES = {
            "chegadinha_frente_1", "prep_coleta_jardim", "coleta_jardim", "lancamento_2",
            "prep_coleta_flower", "coleta_flower", "pos_flower",
            "lancamento_3", "estacionamento"
    };
    private enum Phase {
        WARMING, FOLLOWING, AIMING, SHOOTING, OPENING_FLOWER, COLLECTING_FLOWER,
        CLOSING_FLOWER, DONE
    }

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.4876, 8.9058, 90);
    private final Pose point1 = poseFactory.of(60.2141, 21.4266, 180);
    private final Pose point2Start = poseFactory.of(60.2141, 21.4266, 180);
    private final Pose point2 = poseFactory.of(40.6033, 13.1963, 180);
    private final Pose point3Start = poseFactory.of(40.6033, 13.1963, 180);
    private final Pose point3 = poseFactory.of(15.0925, 13.6014, 180);
    private final Pose point4Start = poseFactory.of(15.0925, 13.6014, 180);
    private final Pose point4 = poseFactory.of(21.1567, 126.6408, -36);
    private final Pose point4Control1 = poseFactory.of(141.5, 94.0755, 0);
    private final Pose point5Start = poseFactory.of(61.2219, 140.4565, -90);
    private final Pose point5 = poseFactory.of(48.3601, 116.8377, 90);
    private final Pose point6Start = poseFactory.of(48.3601, 116.8377, 90);
    private final Pose point6 = poseFactory.of(48.5646, 128.8933, 90);
    private final Pose point7 = poseFactory.of(48.6154, 111.25, 90.1559);
    private final Pose point8Start = poseFactory.of(46.6154, 111.25, -90);
    private final Pose point8 = poseFactory.of(21.1567, 126.6408, 270);
    //private final Pose point9 = poseFactory.of(60.4364, 119.4291, 89.4008);
    //private final Pose point10 = poseFactory.of(8.6851, 119.4314, -0.0025);
    private Follower follower;
    private PedroDrive drive;
    private Intake intake;
    private Flywheel flywheel;
    private Transfer transfer;
    private Trigger trigger;
    private Limelight limelight;
    private FlowerCollect flowerCollect;
    private LongSupplier nanoClock = System::nanoTime;
    private Path[] paths;
    private int pathIndex = -1;
    private int pathGroupStartIndex;
    private int pathGroupEndIndex;
    private int shotNumber;
    private Phase phase = Phase.WARMING;
    private boolean audienceCell;
    private boolean redAlliance;
    private long phaseStartedNanos;
    private long flowerOpenedNanos;
    private String shotStatus = "Aguardando Play";

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
        flowerCollect = new FlowerCollect(hardwareMap, telemetry, true);
        boolean autonomousStarted = false;

        try {
            intake.offMotor();
            setShooting(false);
            follower.setPose(start);
            do {
                follower.update();
                limelight.run(false, true, gamepad2.dpad_up, gamepad2.dpad_down);
                redAlliance = limelight.isRedAlliance();
                telemetry.addLine("V3: disparos no inicio e nos lancamentos 2/3");
                telemetry.addLine("Flower fechada no INIT; coleta parada por 2 s");
                telemetry.addLine("G2 cima: AZUL / baixo: VERMELHA (antes do Play)");
                flowerCollect.sendTelemetry();
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
                flowerCollect.sendTelemetry();
                limelight.sendTelemetry();
                telemetry.update();
                idle();
            }
        } finally {
            finishAutonomous(autonomousStarted);
        }
    }

    void startAutonomous() {
        paths = new Path[] {path1(),path2(),path3(),path4(),path5(),path6(),
        path7(),path8()};
        setShooting(false);
        flowerCollect.setClosed();
        follower.holdEnd.set(true);
        follower.hold(start);
        pathIndex = -1;
        shotNumber = 0;
        audienceCell = false;
        phase = Phase.WARMING;
        phaseStartedNanos = nanoClock.getAsLong();
        shotStatus = "Aquecendo para o lancamento inicial";
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
                if (elapsedMillis(now, phaseStartedNanos) >= SPIN_UP_MS) beginShot(now);
                break;
            case FOLLOWING:
                follower.update();
                updatePathIndex();
                if (!follower.following() && !follower.isBusy()) {
                    if (pathIndex == SECOND_SHOT_PATH || pathIndex == THIRD_SHOT_PATH) {
                        beginShot(now);
                    } else if (pathIndex == PREP_FLOWER_PATH) {
                        phase = Phase.OPENING_FLOWER;
                    } else if (pathIndex == COLLECT_FLOWER_PATH) {
                        phase = Phase.COLLECTING_FLOWER;
                        phaseStartedNanos = now;
                    } else if (pathIndex == POST_FLOWER_PATH) {
                        flowerCollect.setClosed();
                        phase = Phase.CLOSING_FLOWER;
                        phaseStartedNanos = now;
                    } else {
                        advancePath();
                    }
                }
                break;
            case AIMING:
                drive.aimVisionHold(pathIndex < 0 ? start : paths[pathIndex].endPose(),
                        limelight.getObservation());
                if (elapsedMillis(now, phaseStartedNanos) >= AIM_TIME_MS) {
                    // Encerra a correcao visual e mantem o heading medido durante o disparo.
                    Pose shotPose = pathIndex < 0 ? start : paths[pathIndex].endPose();
                    follower.hold(new Pose(shotPose.x(), shotPose.y(), follower.pose().heading()));
                    follower.update();
                    phase = Phase.SHOOTING;
                    phaseStartedNanos = now;
                    shotStatus = "Lancamento " + shotNumber + " em hold por " + SHOOT_TIME_MS + " ms";
                    feeding = true;
                }
                break;
            case SHOOTING:
                follower.update();
                if (elapsedMillis(now, phaseStartedNanos) >= SHOOT_TIME_MS) {
                    shotStatus = "Lancamento " + shotNumber + " concluido";
                    finishShot();
                } else {
                    feeding = true;
                }
                break;
            case OPENING_FLOWER:
                follower.update();
                if (!follower.isBusy()
                        && elapsedMillis(now, flowerOpenedNanos) >= FLOWER_SERVO_SETTLE_MS) {
                    advancePath();
                }
                break;
            case COLLECTING_FLOWER:
                follower.update();
                if (!follower.isBusy()
                        && elapsedMillis(now, phaseStartedNanos) >= FLOWER_COLLECT_MS) {
                    advancePath();
                }
                break;
            case CLOSING_FLOWER:
                follower.update();
                if (!follower.isBusy()
                        && elapsedMillis(now, phaseStartedNanos) >= FLOWER_SERVO_SETTLE_MS) {
                    advancePath();
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

    private void beginShot(long now) {
        shotNumber++;
        phase = Phase.AIMING;
        phaseStartedNanos = now;
        shotStatus = "Mirada breve de 0,5 s para o lancamento " + shotNumber;
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
            pathGroupStartIndex = pathIndex;
            pathGroupEndIndex = pathIndex;
            // O Pedro troca os segmentos sem hold ate o proximo ponto de parada.
            while (pathGroupEndIndex < paths.length - 1 && !requiresStop(pathGroupEndIndex)) {
                pathGroupEndIndex++;
            }
            Path pathGroup = Paths.path(Arrays.copyOfRange(paths,
                    pathGroupStartIndex, pathGroupEndIndex + 1));
            if (pathGroupEndIndex > pathGroupStartIndex || pathIndex == PREP_FLOWER_PATH
                    || pathIndex == THIRD_SHOT_PATH) {
                pathGroup = pathGroup.with(Constants.foresightConfig.brakeAggression
                        .at(TRANSIT_BRAKE_AGGRESSION));
            }
            if (pathIndex == PREP_FLOWER_PATH) {
                flowerCollect.setOpen();
                flowerOpenedNanos = nanoClock.getAsLong();
            }
            follower.follow(pathGroup);
            phase = Phase.FOLLOWING;
        } else {
            pathIndex = paths.length - 1;
            phase = Phase.DONE;
            flywheel.stopMotors();
            intake.offMotor();
        }
    }

    private static boolean requiresStop(int index) {
        return index == SECOND_SHOT_PATH || index == PREP_FLOWER_PATH
                || index == COLLECT_FLOWER_PATH || index == POST_FLOWER_PATH
                || index == THIRD_SHOT_PATH;
    }

    private void updatePathIndex() {
        // O tracker pode apontar alem do ultimo segmento um ciclo antes do hold.
        int currentIndex = follower.following()
                ? Math.min(pathGroupEndIndex, pathGroupStartIndex + follower.pathIndex())
                : pathGroupEndIndex;
        pathIndex = currentIndex;
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
                        try {
                            flowerCollect.setClosed();
                        } finally {
                            limelight.stop();
                        }
                    }
                }
            }
        }
    }

    public Path path1() {
        return Paths.line(start, point1).linear(start, point1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return Paths.line(point6, point7).reverseTangent();
    }

    public Path path8() {
        return Paths.line(point8Start, point8).constant(point8);
    }

    //public Path path9() {
    //    return Paths.line(point8, point9).reverseTangent();
    //}

   // public Path path10() {
   //     return Paths.line(point9, point10).reverseTangent();
   // }
}