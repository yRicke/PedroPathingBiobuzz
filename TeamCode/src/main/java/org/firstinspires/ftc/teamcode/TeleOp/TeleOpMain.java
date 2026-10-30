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

@TeleOp(name="TeleOpMain", group="TeleOpMode")

public class TeleOpMain extends LinearOpMode {

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

        telemetry.addLine("Status: Inicializado");
        telemetry.addLine("Drive: Pedro field-centric com hold automatico");
        telemetry.addLine("Frente do campo: orientacao do robo ao apertar Play");
        telemetry.addLine("D-pad cima: AZUL / baixo: VERMELHA (inicial: AZUL)");
        telemetry.addLine("D-pad esquerda: CELL plateia / direita: CELL oposta (inicial)");
        limelight.sendTelemetry();
        telemetry.addLine("Right bumper: mirar no hive visivel com translacao livre");
        telemetry.addLine("Sem alvo: giro manual, mesmo segurando right bumper");
        telemetry.addLine("Right trigger: disparar");
        telemetry.update();

        try {
            waitForStart();
            if (isStopRequested()) {
                return;
            }

            // A orientacao no Play define o zero do campo para este TeleOp.
            follower.setPose(Pose.zero());

            while (opModeIsActive()) {

                //Subsistemas
                limelight.run(gamepad2.dpad_left, gamepad2.dpad_right,
                        gamepad2.dpad_up, gamepad2.dpad_down);
                boolean aimRequested = gamepad1.right_bumper;
                boolean shootRequested = gamepad1.right_trigger > 0;
                if (aimRequested) {
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
                // Telemetry Organizado
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
