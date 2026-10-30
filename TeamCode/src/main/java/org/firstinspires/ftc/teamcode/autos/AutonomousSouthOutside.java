package org.firstinspires.ftc.teamcode.autos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Trigger.Trigger;

@Autonomous(name = "AutonomousSouthOutside", group = "Autonomous")
public class AutonomousSouthOutside extends LinearOpMode {
    private static final double FLYWHEEL_RPM = 2670;
    private static final double SPIN_UP_MS = 1500;
    private static final double SHOOT_TIME_MS = 3000;

    private Follower follower;
    private Intake intake;
    private Flywheel flywheel;
    private Transfer transfer;
    private Trigger trigger;
    private String stage = "Inicializado";

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 9, 90);
    private final Pose foward = poseFactory.of(56, 15, 90);
    private final Pose path1 = poseFactory.of(31, 15, 180);
    private final Pose point2 = poseFactory.of(12, 15, 180);
    private final Pose point3 = poseFactory.of(56, 133, -90);
    private final Pose point3Control1 = poseFactory.of(79, 1, 0);
    private final Pose point4 = poseFactory.of(4, 116, 0.0923);
    private final Pose point4Control1 = poseFactory.of(57, 116, 0);

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);
        flywheel = new Flywheel(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);
        trigger = new Trigger(hardwareMap, telemetry);

        try {
            intake.offMotor();
            setShooting(false);
            follower.holdEnd.set(true);
            follower.setPose(start);
            follower.update();
            Command routine = autoRoutine();

            telemetry.addLine("Pronto: flywheel 2700 RPM e intake continuo apos o Play");
            telemetry.addLine("Disparos de 5 s em hold: antes de foward e apos path3");
            telemetry.update();

            waitForStart();
            if (isStopRequested()) {
                return;
            }

            schedule(routine);

            while (opModeIsActive()) {
                // O controle de RPM precisa ser recalculado em todos os ciclos,
                // inclusive durante as esperas, os disparos e o hold final.
                flywheel.run(FLYWHEEL_RPM);
                intake.onMotor();
                follower.update();
                Scheduler.execute();

                telemetry.addData("Etapa", stage);
                telemetry.addData("Follower", follower.mode());
                telemetry.addData("X (in)", follower.pose().x());
                telemetry.addData("Y (in)", follower.pose().y());
                telemetry.addData("Heading (graus)", Math.toDegrees(follower.pose().heading()));
                flywheel.sendTelemetry();
                telemetry.update();
                idle();
            }
        } finally {
            // Stop tambem pode ocorrer durante uma espera ou um disparo.
            try {
                Scheduler.reset();
                follower.stop();
                follower.drivetrain.stop();
            } finally {
                try {
                    setShooting(false);
                    intake.offMotor();
                } finally {
                    flywheel.stopMotors();
                }
            }
        }
    }

    Command autoRoutine() {
        return sequential(
                instant(() -> stage = "Aquecendo flywheel em hold"),
                hold(follower, start),
                waitMs(SPIN_UP_MS),
                shoot("Disparo inicial em hold"),
                followPath("foward", foward()),
                followPath("path1", path1()),
                followPath("path2", path2()),
                followPath("path3", path3()),
                // holdEnd conserva a pose final real, incluindo o reverseTangent.
                shoot("Disparo apos path3 em hold"),
                followPath("path4", path4()),
                instant(() -> stage = "Concluido em hold")
        );
    }

    private Command followPath(String name, Path path) {
        return sequential(
                instant(() -> stage = name),
                // Ivy 1.1.1 usa atParametricEnd por padrao. Aguarda tambem a parada.
                follow(follower, path).setDone(() -> !follower.following() && !follower.isBusy())
        );
    }

    private Command shoot(String name) {
        return sequential(
                instant(() -> {
                    stage = name;
                    setShooting(true);
                }),
                waitMs(SHOOT_TIME_MS),
                instant(() -> setShooting(false))
        );
    }

    private void setShooting(boolean enabled) {
        trigger.run(enabled);
        transfer.run(enabled);
    }

    public Path foward() {
        return Paths.line(start, foward).linear(start, foward);
    }

    public Path path1() {
        return Paths.line(foward, path1).linear(foward, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).tangent();
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3).reverseTangent();
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).reverseTangent();
    }
}
