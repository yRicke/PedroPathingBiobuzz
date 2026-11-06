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

import java.util.concurrent.TimeUnit;
import java.util.function.LongSupplier;

/** Disparo inicial na plateia, coleta da flower, segundo disparo e estacionamento. */
@Autonomous(name = "Auto_Terciario", group = "Autonomous")
public class AutoTerciario extends LinearOpMode {
    private static final int PREP_FLOWER_PATH = 0;
    private static final int COLLECT_FLOWER_PATH = 1;
    private static final int POST_FLOWER_PATH = 2;
    private static final int RETURN_SHOT_PATH = 3;
    private static final String[] PATH_NAMES = {
            "preparacao_flower", "coleta_flower", "pos_flower",
            "retorno_lancamento_3", "estacionamento"
    };
    private enum Phase {
        WARMING, FOLLOWING, AIMING, SHOOTING, OPENING_FLOWER, COLLECTING_FLOWER,
        CLOSING_FLOWER, DONE
    }

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(59.6258, 136.8065, 270);
    private final Pose prepColetaFlower = poseFactory.of(46.4006, 120.9045, 90);
    private final Pose prepColetaFlowerControl1 = poseFactory.of(67.8287, 116.669, 0);
    private final Pose coletaFlower = poseFactory.of(46.6, 131, 90);
    private final Pose posFlower = poseFactory.of(46.6, 120, 90);
    private final Pose lancamento3 = poseFactory.of(59.8174, 136.7026, -66.9996);
    private final Pose lancamento3Control1 = poseFactory.of(68.0113, 117.3994, 0);
    private final Pose estacionamentoStart = poseFactory.of(59.8174, 136.7026, 270);
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
    private int shotNumber;
    private Phase phase = Phase.WARMING;
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
                limelight.run(true, false, gamepad2.dpad_up, gamepad2.dpad_down);
                redAlliance = limelight.isRedAlliance();
                telemetry.addLine("Terciario: disparos de 2 s no inicio e apos a flower (CELL plateia)");
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
        // O grupo ate_lancamento_2 do arquivo exportado esta vazio: inicia em hold.
        paths = new Path[] {preparacaoflower(), coletaflower(), posflower(),
                retornolancamento3(), estacionamento()};
        setShooting(false);
        flowerCollect.setClosed();
        follower.holdEnd.set(true);
        follower.hold(start);
        pathIndex = -1;
        shotNumber = 0;
        phase = Phase.WARMING;
        phaseStartedNanos = nanoClock.getAsLong();
        shotStatus = "Aquecendo para o lancamento inicial";
    }

    void updateAutonomous() {
        long now = nanoClock.getAsLong();
        limelight.run(true, false, !redAlliance, redAlliance);
        boolean feeding = false;
        if (phase != Phase.DONE) {
            flywheel.run(FLYWHEEL_RPM);
            intake.onMotor();
        }

        switch (phase) {
            case WARMING:
                follower.update();
                if (elapsedMillis(now, phaseStartedNanos) >= SPIN_UP_MS) beginShot(now);
                break;
            case FOLLOWING:
                follower.update();
                if (!follower.following() && !follower.isBusy()) {
                    if (pathIndex == RETURN_SHOT_PATH) {
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
        advancePath();
    }

    private void advancePath() {
        pathIndex++;
        if (pathIndex < paths.length) {
            Path path = paths[pathIndex];
            if (pathIndex == PREP_FLOWER_PATH || pathIndex == RETURN_SHOT_PATH) {
                path = path.with(Constants.foresightConfig.brakeAggression
                        .at(TRANSIT_BRAKE_AGGRESSION));
            }
            if (pathIndex == PREP_FLOWER_PATH) {
                flowerCollect.setOpen();
                flowerOpenedNanos = nanoClock.getAsLong();
            }
            follower.follow(path);
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

    public Path preparacaoflower() {
        return Paths.path(Paths.curve(start, prepColetaFlowerControl1, prepColetaFlower)
                .linear(start, prepColetaFlower));
    }

    public Path coletaflower() {
        return Paths.line(prepColetaFlower, coletaFlower).constant(coletaFlower);
    }

    public Path posflower() {
        return Paths.line(coletaFlower, posFlower).reverseTangent();
    }

    public Path retornolancamento3() {
        return Paths.path(Paths.curve(posFlower, lancamento3Control1, lancamento3).reverseTangent());
    }

    public Path estacionamento() {
        return Paths.curve(estacionamentoStart, estacionamentoControl1, estacionamento)
                .linear(estacionamentoStart, estacionamento);
    }
}
