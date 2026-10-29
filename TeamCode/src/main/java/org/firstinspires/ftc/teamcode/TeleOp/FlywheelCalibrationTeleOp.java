package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DedinhoNervoso.DedinhoNervoso;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Trigger.Trigger;

import java.util.Locale;
import java.util.OptionalDouble;

/** Calibracao parada: RPM manual, distancia 3D ao centro da CELL e captura. */
@TeleOp(name = "Flywheel Calibration - 3D", group = "Flywheel")
public class FlywheelCalibrationTeleOp extends LinearOpMode {
    private static final int[] STEPS_TENTHS = {1, 10, 100, 1000, 10000};
    private int maxTenths;
    private int targetTenths;
    private int stepIndex = 2;
    private boolean enabled;
    private boolean previousUp, previousDown, previousLeft, previousRight, previousA;

    void resetCalibrationControls(double initialRpm, double maxRpm) {
        if (!Double.isFinite(initialRpm) || !Double.isFinite(maxRpm)
                || initialRpm < 0 || maxRpm <= 0 || initialRpm > maxRpm
                || maxRpm > Integer.MAX_VALUE / 10.0) {
            throw new IllegalArgumentException("Limites de RPM invalidos");
        }
        maxTenths = (int) Math.round(maxRpm * 10);
        targetTenths = (int) Math.round(initialRpm * 10);
        stepIndex = 2;
        enabled = false;
        previousUp = previousDown = previousLeft = previousRight = previousA = false;
    }

    void updateCalibrationControls(boolean up, boolean down, boolean left, boolean right,
                       boolean a, boolean stop) {
        if (left != right) {
            if (left && !previousLeft) stepIndex = Math.max(0, stepIndex - 1);
            if (right && !previousRight) {
                stepIndex = Math.min(STEPS_TENTHS.length - 1, stepIndex + 1);
            }
        }
        if (up != down) {
            if (up && !previousUp) {
                targetTenths = (int) Math.min((long) maxTenths,
                        (long) targetTenths + STEPS_TENTHS[stepIndex]);
            }
            if (down && !previousDown) {
                targetTenths = Math.max(0, targetTenths - STEPS_TENTHS[stepIndex]);
            }
        }
        if (stop) enabled = false;
        else if (a && !previousA) enabled = !enabled;
        previousUp = up;
        previousDown = down;
        previousLeft = left;
        previousRight = right;
        previousA = a;
    }

    double getCalibrationRpm() { return targetTenths / 10.0; }
    double getCalibrationStep() { return STEPS_TENTHS[stepIndex] / 10.0; }
    boolean isFlywheelEnabled() { return enabled; }
    double getCommandedRpm() { return enabled ? getCalibrationRpm() : 0; }

    @Override
    public void runOpMode() {
        resetCalibrationControls(Flywheel.BASE_RPM, Flywheel.MAX_RPM);
        Flywheel flywheel = new Flywheel(hardwareMap, telemetry);
        Transfer transfer = new Transfer(hardwareMap, telemetry);
        Trigger trigger = new Trigger(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);
        DedinhoNervoso dedo = new DedinhoNervoso(hardwareMap, telemetry);
        trigger.setTriggerClosed();
        dedo.setDedinhoNervosoClosed();
        Limelight limelight = new Limelight(hardwareMap, telemetry);
        boolean audienceCell = false;
        boolean previousX = false, previousY = false;
        String capturedPair = "Nenhum: X captura; anote antes de sair";
        String capturedDetails = "Captura fica apenas nesta execucao";

        try {
            telemetry.setMsTransmissionInterval(100);
            telemetry.addLine("Robo parado. A liga/desliga flywheel; B desliga.");
            telemetry.addLine("D-pad cima/baixo: RPM; esquerda/direita: passo.");
            telemetry.addLine("RT dispara; LB alterna intake; RB abre dedinho.");
            telemetry.addLine("Y troca CELL; X congela par para anotar.");
            telemetry.addLine("Clique stick esquerdo: AZUL / direito: VERMELHA.");
            limelight.sendTelemetry();
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            while (opModeIsActive()) {
                updateCalibrationControls(gamepad1.dpad_up, gamepad1.dpad_down,
                        gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.a, gamepad1.b);
                if (gamepad1.y && !previousY) audienceCell = !audienceCell;
                previousY = gamepad1.y;
                limelight.run(audienceCell, !audienceCell,
                        gamepad1.left_stick_button, gamepad1.right_stick_button);
                int firstTagId = limelight.getObservation().getFirstTagId();
                double currentRpm = flywheel.getRPM();
                OptionalDouble distance = limelight.getDistanceMeters();

                if (gamepad1.x && !previousX) {
                    if (isFlywheelEnabled() && getCalibrationRpm() > 0
                            && distance.isPresent() && Double.isFinite(currentRpm)) {
                        capturedPair = String.format(Locale.US,
                                "{%.3f, %.1f}, // centro CELL cm, RPM desejada",
                                distance.getAsDouble() * 100, getCalibrationRpm());
                        capturedDetails = String.format(Locale.US,
                                "%s | CELL %d-%d | RPM medida %.1f | %d amostras",
                                limelight.isRedAlliance() ? "VERMELHA" : "AZUL",
                                firstTagId, firstTagId + Limelight.TAGS_PER_HIVE - 1,
                                currentRpm, limelight.getDistanceSampleCount());
                    } else {
                        capturedDetails = "Captura recusada: ligue flywheel e confira distancia 3D valida";
                    }
                }
                previousX = gamepad1.x;

                flywheel.run(getCommandedRpm());
                boolean shoot = getCommandedRpm() > 0 && gamepad1.right_trigger > 0;
                transfer.run(shoot);
                trigger.run(shoot);
                intake.run(gamepad1.left_bumper, false);
                dedo.run(gamepad1.right_bumper);

                telemetry.addData("Flywheel", isFlywheelEnabled() ? "LIGADA" : "DESLIGADA (A liga)");
                telemetry.addData("RPM desejada", "%.1f", getCalibrationRpm());
                telemetry.addData("Passo RPM", "%.1f", getCalibrationStep());
                flywheel.sendTelemetry();
                limelight.sendTelemetry();
                telemetry.addData("PAR ANOTADO", capturedPair);
                telemetry.addData("Captura", capturedDetails);
                telemetry.addLine("Cima/baixo RPM | Esq/dir passo | A liga | B desliga");
                telemetry.addLine("RT dispara | LB intake | RB dedinho | Y CELL | X captura");
                telemetry.addLine("Clique stick esq: AZUL | dir: VERMELHA");
                telemetry.update();
                idle();
            }
        } finally {
            try {
                flywheel.stopMotors();
            } finally {
                try {
                    transfer.offMotor();
                    intake.offMotor();
                    trigger.setTriggerClosed();
                    dedo.setDedinhoNervosoClosed();
                } finally {
                    limelight.stop();
                }
            }
        }
    }
}
