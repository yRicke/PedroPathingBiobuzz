    package org.firstinspires.ftc.teamcode.TeleOp;

    import com.pedropathing.follower.Follower;
    import com.pedropathing.math.Pose;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

    import org.firstinspires.ftc.teamcode.pedro.Constants;
    import org.firstinspires.ftc.teamcode.subsystems.DedinhoNervoso.DedinhoNervoso;
    import org.firstinspires.ftc.teamcode.subsystems.Drive.PedroDrive;
    import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;
    import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.FlywheelRpmCalculator;
    import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
    import org.firstinspires.ftc.teamcode.subsystems.Limelight.Limelight;
    import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
    import org.firstinspires.ftc.teamcode.subsystems.Trigger.Trigger;

    import java.util.OptionalDouble;
    import java.util.concurrent.TimeUnit;

    @TeleOp(name="TeleOpMainTransferTest", group="TeleOpMode")

    public class TeleOpMainTransferTest extends LinearOpMode {

        // Preencha com tdist FILTRADO (cm) e RPM desejada do TeleOp de calibracao.
        // Distancia zero desativa o par. Os antigos 308 cm NAO eram tdist.
        public static double RPM_BASE = 2720;
        public static double DISTANCIA_BASE_CM = 0;
        public static double RPM_BASE_2 = 0;
        public static double DISTANCIA_BASE_2_CM = 0;
        public static double RPM_BASE_3 = 0;
        public static double DISTANCIA_BASE_3_CM = 0;
        private static final long RPM_UPDATE_NANOS = TimeUnit.MILLISECONDS.toNanos(100);
        private static final double RPM_DEADBAND = 10;
        // true para a camera fisicamente montada de cabeca para baixo.
        private static final boolean LIMELIGHT_UPSIDE_DOWN = true;
        // Configure esta pipeline como AprilTag na interface da Limelight.
        private static final int LIMELIGHT_PIPELINE = 8;

        private PedroDrive drive;
        private Limelight limelight;
        private Intake intake;
        private Flywheel flywheel;
        private Transfer transfer;
        private Trigger trigger;
        private DedinhoNervoso dedo;

        @Override
        public void runOpMode() {
            double[][] calibrationPairs = {
                    {DISTANCIA_BASE_CM, RPM_BASE},
                    {DISTANCIA_BASE_2_CM, RPM_BASE_2},
                    {DISTANCIA_BASE_3_CM, RPM_BASE_3}
            };
            // Valide antes de iniciar hardware/polling da camera.
            int pointCount = FlywheelRpmCalculator.pointCount(calibrationPairs);
            double targetRpm = FlywheelRpmCalculator.calculate(
                    OptionalDouble.empty(), RPM_BASE, calibrationPairs);
            long lastRpmUpdate = System.nanoTime();
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
            limelight = new Limelight(hardwareMap, telemetry,
                    LIMELIGHT_UPSIDE_DOWN, LIMELIGHT_PIPELINE);

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
                    OptionalDouble shotDistance = limelight.getTdistMeters();
                    double requestedRpm = FlywheelRpmCalculator.calculate(
                            shotDistance, RPM_BASE, calibrationPairs);
                    long now = System.nanoTime();
                    if (requestedRpm <= 0 || (now - lastRpmUpdate >= RPM_UPDATE_NANOS
                            && Math.abs(requestedRpm - targetRpm) >= RPM_DEADBAND)) {
                        targetRpm = requestedRpm;
                        lastRpmUpdate = now;
                    }
                    // O controle do motor continua rodando em TODOS os ciclos.
                    flywheel.run(targetRpm);
                    transfer.run(shootRequested, gamepad1.left_bumper);
                    trigger.run(shootRequested);
                    dedo.run(gamepad1.b);
                    // Telemetry Organizado
                    drive.sendTelemetry();
                    limelight.sendTelemetry();
                    intake.sendTelemetry();
                    flywheel.sendTelemetry();
                    telemetry.addData("Pares RPM calibrados", pointCount);
                    telemetry.addData("RPM calculada antes da banda morta", requestedRpm);
                    telemetry.addData("Modo RPM", !shotDistance.isPresent()
                            ? "Sem tdist: RPM_BASE fixa"
                            : pointCount < 2 ? "Menos de 2 pares: RPM fixa"
                            : FlywheelRpmCalculator.isInCalibratedRange(shotDistance, calibrationPairs)
                            ? "Interpolacao tdist" : "Fora da faixa: RPM limitada ao extremo");
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
                            trigger.setTriggerClosed();
                        } finally {
                            flywheel.stopMotors();
                        }
                    }
                }
            }
        }
    }
