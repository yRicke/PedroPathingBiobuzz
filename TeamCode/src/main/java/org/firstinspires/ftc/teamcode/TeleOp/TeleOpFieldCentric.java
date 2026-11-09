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
import org.firstinspires.ftc.teamcode.subsystems.Limelight.AprilTagObservation;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Trigger.Trigger;
import org.firstinspires.ftc.teamcode.subsystems.FlowerCollect.FlowerCollect;

/** TeleOp unico com localizacao do Pedro e ajuste final de mira pela Limelight. */
@TeleOp(name = "TeleOpFieldCentric", group = "TeleOpMode")
public class TeleOpFieldCentric extends LinearOpMode {
    // Referencial adotado para a alianca atual: X/Y em polegadas, heading em radianos.
    private static final Pose INITIAL_POSE = new Pose(8.4, 35.2, Math.toRadians(0));
    private static final Pose RESET_POSE = new Pose(8.4, 35.2, Math.toRadians(0));

    private PedroDrive drive;
    private Limelight limelight;
    private Intake intake;
    private Flywheel flywheel;
    private Transfer transfer;
    private Trigger trigger;
    private FlowerCollect flowerCollect;
//<<<<<<< HEAD

//=======
//>>>>>>> d06ca30eaecb8a46b87eb487276679374d961ce3
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
        flowerCollect = new FlowerCollect(hardwareMap,telemetry);

        try {
            Pose autonomousPose = OpModeStorage.consumeAutonomousPose();
            boolean restoredFromAuto = autonomousPose != null;
            drive.resetPose(restoredFromAuto ? autonomousPose : INITIAL_POSE);
            do {
                // Atualiza a localizacao durante INIT e preserva o heading recebido no Play.
                follower.update();
                limelight.run(gamepad2.dpad_left, gamepad2.dpad_right,
                        gamepad2.dpad_up, gamepad2.dpad_down);
                sendInitTelemetry(restoredFromAuto);
                idle();
            } while (opModeInInit());

            waitForStart();
            if (isStopRequested()) {
                return;
            }

            boolean previousFieldReset = gamepad1.y;
            boolean previousPoseReset = gamepad1.x;

            while (opModeIsActive()) {
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
                AprilTagObservation observation = limelight.getObservation();
                boolean aimRequested = gamepad1.right_bumper;
                boolean shootRequested = gamepad1.right_trigger > 0;
                if (aimRequested) {
                    drive.aimVision(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                            gamepad1.right_stick_x, gamepad1.left_trigger > 0,
                            observation);
                } else {
                    drive.run(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                            gamepad1.right_stick_x, gamepad1.left_trigger > 0);
                }
                intake.run(gamepad1.a, gamepad1.left_bumper);
                flywheel.runByDistance(limelight.getDistanceMeters());
                transfer.run(shootRequested, gamepad1.left_bumper);
                trigger.run(shootRequested);
                flowerCollect.run(gamepad1.b);
                drive.sendTelemetry();
                limelight.sendTelemetry();
                intake.sendTelemetry();
                flywheel.sendTelemetry();
                transfer.sendTelemetry();
                trigger.sendTelemetry();
                flowerCollect.sendTelemetry();
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
        telemetry.addData("Origem da pose", restoredFromAuto ? "Final do autonomo" : "INITIAL_POSE");
        telemetry.addLine("RB: mira Limelight | Sem alvo: giro manual | LT: precisao | RT: disparar");
        telemetry.addLine("A: intake | LB: inverter intake/transfer");
        telemetry.addLine("Y: frente atual dos controles | X: RESET_POSE e frente global");
        telemetry.addLine("G2 cima/baixo: azul/vermelha | esquerda/direita: CELL plateia/oposta");
        telemetry.addData("Pose inicial sem autonomo (in, in, rad)", INITIAL_POSE);
        telemetry.addData("Pose de reset (in, in, rad)", RESET_POSE);
        drive.sendTelemetry();
        limelight.sendTelemetry();
        telemetry.update();
    }
}
