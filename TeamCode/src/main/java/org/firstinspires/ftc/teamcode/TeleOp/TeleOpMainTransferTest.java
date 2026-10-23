    package org.firstinspires.ftc.teamcode.TeleOp;

    import com.pedropathing.follower.Follower;
    import com.pedropathing.math.Pose;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    import org.firstinspires.ftc.teamcode.pedro.Constants;
    import org.firstinspires.ftc.teamcode.subsystems.Drive.PedroDrive;
    import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;
    import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
    import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
    import org.firstinspires.ftc.teamcode.subsystems.Trigger.Trigger;

    @TeleOp(name="TeleOpMainTransferTest", group="TeleOpMode")

    public class TeleOpMainTransferTest extends LinearOpMode {

        private static final double FLYWHEEL_TARGET_RPM = 2700;

        private PedroDrive drive;
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

            telemetry.addLine("Status: Inicializado");
            telemetry.addLine("Drive: Pedro field-centric com hold automatico");
            telemetry.addLine("Frente do campo: orientacao do robo ao apertar Play");
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
                    drive.run(-gamepad1.left_stick_y,gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_trigger > 0, gamepad2.right_stick_x);
                    intake.run(gamepad1.a, gamepad1.left_bumper);
                    flywheel.run(FLYWHEEL_TARGET_RPM);
                    transfer.run(gamepad1.right_trigger > 0, gamepad1.left_bumper);
                    trigger.run(gamepad1.right_trigger > 0);
                    // Telemetry Organizado
                    drive.sendTelemetry();
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
                    trigger.setTriggerClosed();
                    flywheel.stopMotors();
                }
            }
        }
    }
