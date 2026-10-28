package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DedinhoNervoso.DedinhoNervoso;
import org.firstinspires.ftc.teamcode.subsystems.Drive.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.FlywheelRpmController;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Trigger.Trigger;

@TeleOp(name="TeleOpMainTransferTest", group="TeleOpMode")

public class TeleOpMainTransferTest extends LinearOpMode {

    private PedroDrive drive;
    private Limelight limelight;
    private Intake intake;
    private Flywheel flywheel;
    private FlywheelRpmController rpmController;
    private Transfer transfer;
    private Trigger trigger;
    private DedinhoNervoso dedo;

    @Override
    public void runOpMode() {
        rpmController = new FlywheelRpmController(telemetry);
        Follower follower = Constants.create(hardwareMap);
        drive = new PedroDrive(follower, telemetry,
                Constants.foresightConfig.headingFeedback.get());
        intake = new Intake(hardwareMap, telemetry);
        flywheel = new Flywheel(hardwareMap, telemetry);
        transfer = new Transfer(hardwareMap, telemetry);
        trigger = new Trigger(hardwareMap, telemetry);
        dedo = new DedinhoNervoso(hardwareMap, telemetry);
        dedo.setDedinhoNervosoClosed();
        trigger.setTriggerClosed();
        limelight = new Limelight(hardwareMap, telemetry);

        telemetry.addLine("Status: Inicializado");
        telemetry.addLine("Drive: Pedro field-centric com hold automatico");
        telemetry.addLine("Frente do campo: orientacao do robo ao apertar Play");
        telemetry.addLine("D-pad: hive esquerdo 38-41 / direito 42-45");
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
                limelight.run(gamepad1.dpad_left, gamepad1.dpad_right);
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
                flywheel.run(rpmController.run(limelight.getDistanceMeters()));
                transfer.run(shootRequested, gamepad1.left_bumper);
                trigger.run(shootRequested);
                dedo.run(gamepad1.b);
                // Telemetry Organizado
                drive.sendTelemetry();
                limelight.sendTelemetry();
                intake.sendTelemetry();
                flywheel.sendTelemetry();
                rpmController.sendTelemetry();
                transfer.sendTelemetry();
                trigger.sendTelemetry();
                dedo.sendTelemetry();
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
                        dedo.setDedinhoNervosoClosed();
                        trigger.setTriggerClosed();
                    } finally {
                        flywheel.stopMotors();
                        rpmController.reset();
                    }
                }
            }
        }
    }
}
