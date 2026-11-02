package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        telemetry.addLine("Status: Inicializado - teste de localizacao Pedro");
        telemetry.addLine("Drive: field-centric com hold automatico; heading global do campo");
        telemetry.addLine("Gamepad 1 Y: frente atual vira referencia dos controles");
        telemetry.addLine("Gamepad 1 X: relocalizar na RESET_POSE conhecida e restaurar frente global");
        telemetry.addData("Pose inicial (in, in, rad)", INITIAL_POSE);
        telemetry.addData("Pose de reset (in, in, rad)", RESET_POSE);
        telemetry.addLine("Gamepad 2 cima/baixo: AZUL/VERMELHA; esquerda/direita: CELL plateia/oposta");
        telemetry.addLine("Right bumper: mirar com Limelight; right trigger: disparar");
        telemetry.addLine("Left trigger: precisao; A: intake; left bumper: inverter intake/transfer");
        limelight.sendTelemetry();
        telemetry.update();

        try {
            waitForStart();
            if (isStopRequested()) {
                return;
            }

            drive.resetPose(INITIAL_POSE);
            boolean previousFieldReset = gamepad1.y;
            boolean previousPoseReset = gamepad1.x;

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
                boolean shootRequested = gamepad1.right_trigger > 0;
                if (gamepad1.right_bumper) {
                    drive.aim(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                            gamepad1.right_stick_x, gamepad1.left_trigger > 0,
                            limelight.getObservation());
                } else {
                    drive.run(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                            gamepad1.right_stick_x, gamepad1.left_trigger > 0);
                }
                intake.run(gamepad1.a, gamepad1.left_bumper);
                flywheel.runByDistance(limelight.getDistanceMeters());
                transfer.run(shootRequested, gamepad1.left_bumper);
                trigger.run(shootRequested);

                drive.sendTelemetry();
                limelight.sendTelemetry();
                intake.sendTelemetry();
                flywheel.sendTelemetry();
                transfer.sendTelemetry();
                trigger.sendTelemetry();
                telemetry.update();
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
}
