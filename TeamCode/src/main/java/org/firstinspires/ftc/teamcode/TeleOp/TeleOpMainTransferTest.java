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

    @TeleOp(name="TeleOpMainTransferTest", group="TeleOpMode")

    public class TeleOpMainTransferTest extends LinearOpMode {

        private static final double FLYWHEEL_TARGET_RPM = 2700;
        // true para a camera fisicamente montada de cabeca para baixo.
        private static final boolean LIMELIGHT_UPSIDE_DOWN = false;
        // Configure esta pipeline como AprilTag na interface da Limelight.
        private static final int LIMELIGHT_PIPELINE = 0;

        private PedroDrive drive;
        private Limelight limelight;
        private Intake intake;
        private Flywheel flywheel;
        private Transfer transfer;
        private Trigger trigger;

        @Override
        public void runOpMode() {
            Follower follower = Constants.create(hardwareMap);
            drive = new PedroDrive(follower, telemetry);
            intake = new Intake(hardwareMap, telemetry);
            flywheel = new Flywheel(hardwareMap, telemetry);
            transfer = new Transfer(hardwareMap, telemetry);
            trigger = new Trigger(hardwareMap, telemetry);
            trigger.setTriggerClosed();
            limelight = new Limelight(hardwareMap, telemetry,
                    LIMELIGHT_UPSIDE_DOWN, LIMELIGHT_PIPELINE);

            telemetry.addLine("Status: Inicializado");
            telemetry.addLine("Drive: Pedro field-centric com hold automatico");
            telemetry.addLine("Frente do campo: orientacao do robo ao apertar Play");
            telemetry.addLine("D-pad: esquerda tag 39 / direita tag 43");
            telemetry.addLine("Right bumper: buscar e manter AprilTag centralizada");
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
                        drive.aim(limelight.getObservation());
                    } else {
                        drive.run(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                                gamepad1.right_stick_x, gamepad1.left_trigger > 0);
                    }
                    intake.run(gamepad1.a, gamepad1.left_bumper);
                    flywheel.run(FLYWHEEL_TARGET_RPM);
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
                            trigger.setTriggerClosed();
                        } finally {
                            flywheel.stopMotors();
                        }
                    }
                }
            }
        }
    }
