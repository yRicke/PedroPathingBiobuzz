package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Trigger.Trigger;

/** Variante de teste do TeleOpMain com coordenadas globais e relocalizacao. */
@TeleOp(name = "TeleOpMainPedro", group = "TeleOpMode")
public class TeleOpMainPedro extends LinearOpMode {
    // Coordenadas em polegadas; heading em radianos. Ajustar para a montagem no campo.
    private static final Pose INITIAL_POSE = new Pose(8.4, 35.2, Math.toRadians(0));
    private static final Pose RESET_POSE = new Pose(8.4, 35.2, Math.toRadians(0));
    private static final Pose RIGHT_HIVE = new Pose(57.8, 50);
    private static final Pose LEFT_HIVE = new Pose(57.8, 90);

    private PedroDrive drive;
    private Limelight limelight;
    private Intake intake;
    private Flywheel flywheel;
    private Transfer transfer;
    private Trigger trigger;

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        drive = new PedroDrive(follower, telemetry,
                Constants.foresightConfig.headingFeedback.get());
        intake = new Intake(hardwareMap, telemetry);
        flywheel = new Flywheel(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);
        trigger = new Trigger(hardwareMap, telemetry);
        trigger.setTriggerClosed();
        limelight = new Limelight(hardwareMap, telemetry);

        try {
            Pose autonomousPose = OpModeStorage.consumeAutonomousPose();
            boolean restoredFromAuto = autonomousPose != null;
            // A referencia dos sticks permanece no zero global, inclusive com heading de 23 graus.
            drive.resetPose(restoredFromAuto ? autonomousPose : INITIAL_POSE);
            do {
                // Acompanha tambem deslocamentos feitos entre INIT e Play, sem comandar movimento.
                follower.update();
                sendInitTelemetry(restoredFromAuto);
                idle();
            } while (opModeInInit());

            waitForStart();
            if (isStopRequested()) {
                return;
            }

            boolean previousFieldReset = gamepad1.y;
            boolean previousPoseReset = gamepad1.x;
            boolean rightHiveSelected = true;

            while (opModeIsActive()) {
                // Bordas: manter o botao pressionado nao recalibra a cada ciclo.
                boolean poseReset = gamepad1.x;
                boolean fieldReset = gamepad1.y;
                if (poseReset && !previousPoseReset) {
                    drive.resetPose(RESET_POSE);
                } else if (fieldReset && !previousFieldReset) {
                    drive.resetFieldCentric();
                }
                previousPoseReset = poseReset;
                previousFieldReset = fieldReset;

                limelight.run(gamepad2.dpad_left, gamepad2.dpad_right,
                        gamepad2.dpad_up, gamepad2.dpad_down);
                if (gamepad2.dpad_left != gamepad2.dpad_right) {
                    rightHiveSelected = gamepad2.dpad_right;
                }
                Pose target = rightHiveSelected ? RIGHT_HIVE : LEFT_HIVE;
                boolean shootRequested = gamepad1.right_trigger > 0;
                if (gamepad1.right_bumper) {
                    drive.aimAt(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                            gamepad1.right_stick_x, gamepad1.left_trigger > 0,
                            target.x(), target.y());
                } else {
                    drive.run(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                            gamepad1.right_stick_x, gamepad1.left_trigger > 0);
                }
                intake.run(gamepad1.a, gamepad1.left_bumper);
                flywheel.runByDistance(limelight.getDistanceMeters());
                transfer.run(shootRequested, gamepad1.left_bumper);
                trigger.run(shootRequested);

                drive.sendTelemetry();
                telemetry.addData("Hive mira Pedro", rightHiveSelected ? "Direito" : "Esquerdo");
                telemetry.addData("Hive X (in)", target.x());
                telemetry.addData("Hive Y (in)", target.y());
                limelight.sendTelemetry();
                intake.sendTelemetry();
                flywheel.sendTelemetry();
                transfer.sendTelemetry();
                trigger.sendTelemetry();
                telemetry.update();
                idle();
            }
        } finally {
            try {
                drive.stop();
            } finally {
                try {
                    limelight.stop();
                } finally {
                    try {
                        transfer.offMotor();
                        intake.offMotor();
                        trigger.setTriggerClosed();
                    } finally {
                        flywheel.stopMotors();
                    }
                }
            }
        }
    }

    private void sendInitTelemetry(boolean restoredFromAuto) {
        telemetry.addLine("Status: Inicializado - teste de localizacao Pedro");
        telemetry.addData("Origem da pose", restoredFromAuto ? "Final do autonomo" : "INITIAL_POSE");
        telemetry.addLine("Drive: field-centric com hold automatico; heading global do campo");
        telemetry.addLine("Gamepad 1 Y: frente atual vira referencia dos controles");
        telemetry.addLine("Gamepad 1 X: relocalizar na RESET_POSE conhecida e restaurar frente global");
        telemetry.addData("Pose inicial sem autonomo (in, in, rad)", INITIAL_POSE);
        telemetry.addData("Pose de reset (in, in, rad)", RESET_POSE);
        telemetry.addLine("Gamepad 2 esquerda/direita: hive esquerdo/direito (inicial: direito)");
        telemetry.addLine("Gamepad 2 cima/baixo: AZUL/VERMELHA para distancia da Limelight");
        telemetry.addLine("Right bumper: mirar por coordenadas Pedro; right trigger: disparar");
        telemetry.addLine("Left trigger: precisao; A: intake; left bumper: inverter intake/transfer");
        drive.sendTelemetry();
        limelight.sendTelemetry();
        telemetry.update();
    }
}
