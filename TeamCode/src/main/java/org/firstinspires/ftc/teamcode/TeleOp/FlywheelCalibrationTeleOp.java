package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Trigger.Trigger;

import java.util.OptionalDouble;

/** Calibracao parada com anotacao manual da distancia e RPM. */
@TeleOp(name = "Flywheel Calibration - 3D", group = "Flywheel")
public class FlywheelCalibrationTeleOp extends LinearOpMode {
    private static final int[] STEPS_TENTHS = {1, 10, 100, 1000, 10000};
    private int minTenths;
    private int maxTenths;
    private int targetTenths;
    private int stepIndex = 2;
    private boolean flywheelEnabled, intakeEnabled, transferEnabled, triggerEnabled;
    private boolean reversing;
    private boolean previousUp, previousDown, previousLeft, previousRight;
    private boolean previousFlywheel, previousIntake;

    void resetCalibrationControls(double initialRpm, double minRpm, double maxRpm) {
        if (!Double.isFinite(initialRpm) || !Double.isFinite(minRpm) || !Double.isFinite(maxRpm)
                || minRpm < 0 || initialRpm < minRpm || maxRpm <= 0 || initialRpm > maxRpm
                || maxRpm > Integer.MAX_VALUE / 10.0) {
            throw new IllegalArgumentException("Limites de RPM invalidos");
        }
        minTenths = (int) Math.round(minRpm * 10);
        maxTenths = (int) Math.round(maxRpm * 10);
        targetTenths = (int) Math.round(initialRpm * 10);
        stepIndex = 2;
        flywheelEnabled = intakeEnabled = transferEnabled = triggerEnabled = false;
        reversing = false;
        previousUp = previousDown = previousLeft = previousRight = false;
        previousFlywheel = previousIntake = false;
    }

    void updateCalibrationControls(boolean up, boolean down, boolean left, boolean right,
                                   boolean flywheel, boolean intake, boolean shoot, boolean eject) {
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
                targetTenths = Math.max(minTenths, targetTenths - STEPS_TENTHS[stepIndex]);
            }
        }
        if (flywheel && !previousFlywheel) flywheelEnabled = !flywheelEnabled;
        if (intake && !previousIntake) intakeEnabled = !intakeEnabled;
        // Mesma alimentacao do TeleOpMain: RT momentaneo e LB com prioridade na reversao.
        transferEnabled = shoot;
        triggerEnabled = shoot;
        reversing = eject;
        previousUp = up;
        previousDown = down;
        previousLeft = left;
        previousRight = right;
        previousFlywheel = flywheel;
        previousIntake = intake;
    }

    double getCalibrationRpm() { return targetTenths / 10.0; }
    double getCalibrationStep() { return STEPS_TENTHS[stepIndex] / 10.0; }
    boolean isFlywheelEnabled() { return flywheelEnabled; }
    boolean isIntakeEnabled() { return intakeEnabled; }
    boolean isTransferEnabled() { return transferEnabled; }
    boolean isTriggerEnabled() { return triggerEnabled; }
    boolean isReversing() { return reversing; }
    double getCommandedRpm() { return flywheelEnabled ? getCalibrationRpm() : 0; }

    @Override
    public void runOpMode() {
        resetCalibrationControls(Flywheel.BASE_RPM, Flywheel.MIN_RPM, Flywheel.MAX_RPM);
        Flywheel flywheel = new Flywheel(hardwareMap, telemetry);
        Transfer transfer = new Transfer(hardwareMap, telemetry);
        Trigger trigger = new Trigger(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);
        intake.offMotor();
        trigger.setTriggerClosed();
        Limelight limelight = new Limelight(hardwareMap, telemetry);

        try {
            telemetry.setMsTransmissionInterval(100);
            do {
                limelight.run(gamepad2.dpad_left, gamepad2.dpad_right,
                        gamepad2.dpad_up, gamepad2.dpad_down);
                telemetry.addLine("Anote distancia/RPM manualmente. Robo parado.");
                telemetry.addLine("G2 D-pad: cima azul / baixo vermelha / esquerda plateia / direita oposta");
                sendCalibrationTelemetry(flywheel, limelight);
                idle();
            } while (opModeInInit());
            waitForStart();
            if (isStopRequested()) return;

            while (opModeIsActive()) {
                updateCalibrationControls(gamepad1.dpad_up, gamepad1.dpad_down,
                        gamepad1.dpad_left, gamepad1.dpad_right,
                        gamepad1.b, gamepad1.a, gamepad1.right_trigger > 0, gamepad1.left_bumper);
                limelight.run(gamepad2.dpad_left, gamepad2.dpad_right,
                        gamepad2.dpad_up, gamepad2.dpad_down);

                flywheel.run(getCommandedRpm());
                if (reversing) intake.eject();
                else if (intakeEnabled) intake.onMotor();
                else intake.offMotor();
                transfer.run(transferEnabled, reversing);
                trigger.run(triggerEnabled);

                sendCalibrationTelemetry(flywheel, limelight);
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
                } finally {
                    limelight.stop();
                }
            }
        }
    }

    private void sendCalibrationTelemetry(Flywheel flywheel, Limelight limelight) {
        OptionalDouble distance = limelight.getDistanceMeters();
        if (distance.isPresent()) {
            telemetry.addData("Distancia abertura (cm)", "%.1f", distance.getAsDouble() * 100);
        } else {
            telemetry.addData("Distancia abertura (cm)", "Sem leitura");
        }
        telemetry.addData("RPM desejada / medida", "%.1f / %.1f", getCalibrationRpm(), flywheel.getRPM());
        telemetry.addData("Passo RPM", "%.1f", getCalibrationStep());
        telemetry.addData("Intake (A/LB) | Flywheel (B)", "%s | %s",
                reversing ? "REV" : intakeEnabled ? "ON" : "OFF", flywheelEnabled ? "ON" : "OFF");
        telemetry.addData("RT Transfer / Trigger", "%s / %s",
                reversing ? "REV" : transferEnabled ? "ON" : "OFF", triggerEnabled ? "ABERTO" : "FECHADO");
        int firstTagId = limelight.getObservation().getFirstTagId();
        boolean audience = firstTagId == Limelight.RED_AUDIENCE_FIRST_TAG
                || firstTagId == Limelight.BLUE_AUDIENCE_FIRST_TAG;
        telemetry.addData("Alvo (G2)", "%s | %s | %d-%d",
                limelight.isRedAlliance() ? "VERMELHA" : "AZUL",
                audience ? "plateia" : "oposta", firstTagId, firstTagId + Limelight.TAGS_PER_HIVE - 1);
        telemetry.addLine("D-pad: cima/baixo RPM | esquerda/direita passo | LB inverte intake/transfer");
        telemetry.update();
    }
}
