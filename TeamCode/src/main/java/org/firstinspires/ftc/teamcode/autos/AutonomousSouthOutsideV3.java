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
import org.firstinspires.ftc.teamcode.subsystems.FlowerCollect.FlowerCollect;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Trigger.Trigger;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import java.util.function.LongSupplier;

/** Tres lancamentos em hold e coleta da flower no percurso do Pedro Visualizer. */
@Autonomous(name = "AutonomousSouthOutsideV3", group = "Autonomous")
public class AutonomousSouthOutsideV3 extends LinearOpMode {
    // Ajuste pequeno nos deslocamentos; 1.0 restaura a frenagem padrao.
    private static final double TRANSIT_BRAKE_AGGRESSION = 1.05;
    private static final long SPIN_UP_MS = 1500;
    private static final long AIM_TIME_MS = 500;
    private static final long SHOOT_TIME_MS = 2000;
    private static final long FLOWER_COLLECT_MS = 2000;
    // Servos posicionais nao informam a chegada: reserva tempo para o movimento.
    private static final long FLOWER_SERVO_SETTLE_MS = 500;

    private static final int SECOND_SHOT_PATH = 3;
    private static final int PREP_FLOWER_PATH = 5;
    private static final int COLLECT_FLOWER_PATH = 6;
    private static final int POST_FLOWER_PATH = 7;
    private static final int THIRD_SHOT_PATH = 9;
    private static final String[] PATH_NAMES = {
            "chegadinha_frente_1", "prep_coleta_jardim", "coleta_jardim", "lancamento_2",
            "chegadinha_frente_2", "prep_coleta_flower", "coleta_flower", "pos_flower",
            "pre_lancamento_3", "lancamento_3", "estacionamento"
    };
    private enum Phase {
        WARMING, FOLLOWING, AIMING, SHOOTING, OPENING_FLOWER, COLLECTING_FLOWER,
        CLOSING_FLOWER, DONE
    }

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(57.8, 9, 90);
    private final Pose chegadinhaFrente1 = poseFactory.of(57.8, 20, 90);
    private final Pose prepColetaJardim = poseFactory.of(31, 13, 180);
    private final Pose coletaJardim = poseFactory.of(14, 13, 180);
    private final Pose lancamento2 = poseFactory.of(60, 145, 270);
    private final Pose lancamentoControl1 = poseFactory.of(38.1594, 48.3839, 0);
    private final Pose lancamentoControl2 = poseFactory.of(6.5729, 108.8181, 0);
    private final Pose lancamentoControl3 = poseFactory.of(57.3303, 105.8968, 0);
    private final Pose chegadinhaFrente2 = poseFactory.of(60, 122, 270);
    private final Pose prepColetaFlower = poseFactory.of(50.6, 122, 90);
    private final Pose coletaFlower = poseFactory.of(46.6, 130, 90);
    private final Pose posFlower = poseFactory.of(46.6, 120, 90);
    private final Pose prepLancamento3 = poseFactory.of(60, 122, 270);
    private final Pose lancamento3 = poseFactory.of(60, 145, 270);
    private final Pose estacionamento = poseFactory.of(10, 120, 0);
    private final Pose estacionamentoControl1 = poseFactory.of(65.729, 114.1129, 0);

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
                telemetry.addLine("V3: disparos de 2 s no inicio e nos lancamentos 2/3");
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
        paths = new Path[] {chegadinhafrente1(), prepcoletajardim(), coletajardim(),
                lancamento2(), chegadinhafrente2(), prepcoletaflower(), coletaflower(),
                posflower(), preplancamento3(), lancamento3(), estacionamento()};
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
                    shotStatus = "Lancamento " + shotNumber + " em hold por 2 s";
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
            if (pathGroupEndIndex > pathGroupStartIndex) {
                pathGroup = pathGroup.with(Constants.foresightConfig.brakeAggression
                        .at(TRANSIT_BRAKE_AGGRESSION));
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
        if (pathIndex < PREP_FLOWER_PATH && currentIndex >= PREP_FLOWER_PATH) {
            flowerCollect.setOpen();
            flowerOpenedNanos = nanoClock.getAsLong();
        }
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

    public Path chegadinhafrente1() { return Paths.line(start, chegadinhaFrente1).tangent(); }
    public Path prepcoletajardim() {
        return Paths.line(chegadinhaFrente1, prepColetaJardim).linear(chegadinhaFrente1, prepColetaJardim);
    }
    public Path coletajardim() { return Paths.line(prepColetaJardim, coletaJardim).constant(coletaJardim); }
    public Path lancamento2() {
        return Paths.curve(coletaJardim, lancamentoControl1, lancamentoControl2,
                lancamentoControl3, lancamento2).linear(coletaJardim, lancamento2);
    }
    public Path chegadinhafrente2() { return Paths.line(lancamento2, chegadinhaFrente2).constant(chegadinhaFrente2); }
    public Path prepcoletaflower() {
        return Paths.line(chegadinhaFrente2, prepColetaFlower).linear(chegadinhaFrente2, prepColetaFlower);
    }
    public Path coletaflower() { return Paths.line(prepColetaFlower, coletaFlower).constant(coletaFlower); }
    public Path posflower() { return Paths.line(coletaFlower, posFlower).reverseTangent(); }
    public Path preplancamento3() {
        return Paths.line(posFlower, prepLancamento3).linear(posFlower, prepLancamento3);
    }
    public Path lancamento3() { return Paths.line(prepLancamento3, lancamento3).reverseTangent(); }
    public Path estacionamento() {
        return Paths.curve(lancamento3, estacionamentoControl1, estacionamento).linear(lancamento3, estacionamento);
    }
}
