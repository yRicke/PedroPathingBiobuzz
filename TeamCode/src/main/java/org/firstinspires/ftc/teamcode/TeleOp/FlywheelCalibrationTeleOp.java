package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DedinhoNervoso.DedinhoNervoso;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.FlywheelConfig;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.FlywheelCalibrationControls;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.LimelightConfig;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Trigger.Trigger;

import java.util.Locale;
import java.util.OptionalDouble;

/** Calibracao com robo parado: RPM manual, leitura tdist e captura para anotacao. */
@TeleOp(name = "Flywheel Calibration - tdist", group = "Flywheel")
public class FlywheelCalibrationTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        FlywheelCalibrationControls controls = new FlywheelCalibrationControls(
                FlywheelConfig.BASE_RPM, FlywheelConfig.MAX_RPM);
        Flywheel flywheel = new Flywheel(hardwareMap, telemetry);
        Transfer transfer = new Transfer(hardwareMap, telemetry);
        Trigger trigger = new Trigger(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);
        DedinhoNervoso dedo = new DedinhoNervoso(hardwareMap, telemetry);
        trigger.setTriggerClosed();
        dedo.setDedinhoNervosoClosed();
        Limelight limelight = new Limelight(hardwareMap, telemetry);
        boolean leftHive = false;
        boolean previousX = false, previousY = false;
        String capturedPair = "Nenhum: X captura; anote antes de sair";
        String capturedDetails = "Captura fica apenas nesta execucao";

        try {
            telemetry.setMsTransmissionInterval(100);
            telemetry.addLine("Robo parado. A liga/desliga flywheel; B desliga.");
            telemetry.addLine("D-pad cima/baixo: RPM; esquerda/direita: passo.");
            telemetry.addLine("RT dispara; LB alterna intake; RB abre dedinho.");
            telemetry.addLine("Y troca hive; X congela par para anotar.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            while (opModeIsActive()) {
                controls.update(gamepad1.dpad_up, gamepad1.dpad_down,
                        gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.a, gamepad1.b);
                if (gamepad1.y && !previousY) leftHive = !leftHive;
                previousY = gamepad1.y;
                limelight.run(leftHive, !leftHive);
                int firstTagId = limelight.getObservation().getFirstTagId();
                double currentRpm = flywheel.getRPM();
                OptionalDouble distance = limelight.getDistanceMeters();

                if (gamepad1.x && !previousX) {
                    if (controls.isEnabled() && controls.getTargetRpm() > 0
                            && distance.isPresent() && Double.isFinite(currentRpm)) {
                        capturedPair = String.format(Locale.US,
                                "{%.3f, %.1f}, // tdist cm, RPM desejada",
                                distance.getAsDouble() * 100, controls.getTargetRpm());
                        capturedDetails = String.format(Locale.US,
                                "Hive %d-%d | RPM medida %.1f | %d amostras",
                                firstTagId, firstTagId + LimelightConfig.TAGS_PER_HIVE - 1,
                                currentRpm, limelight.getDistanceSampleCount());
                    } else {
                        capturedDetails = "Captura recusada: ligue flywheel e confira tdist valido";
                    }
                }
                previousX = gamepad1.x;

                flywheel.run(controls.getCommandedRpm());
                boolean shoot = controls.getCommandedRpm() > 0 && gamepad1.right_trigger > 0;
                transfer.run(shoot);
                trigger.run(shoot);
                intake.run(gamepad1.left_bumper, false);
                dedo.run(gamepad1.right_bumper);

                telemetry.addData("Flywheel", controls.isEnabled() ? "LIGADA" : "DESLIGADA (A liga)");
                telemetry.addData("RPM desejada", "%.1f", controls.getTargetRpm());
                telemetry.addData("Passo RPM", "%.1f", controls.getStepRpm());
                flywheel.sendTelemetry();
                limelight.sendTelemetry();
                telemetry.addData("PAR ANOTADO", capturedPair);
                telemetry.addData("Captura", capturedDetails);
                telemetry.addLine("Cima/baixo RPM | Esq/dir passo | A liga | B desliga");
                telemetry.addLine("RT dispara | LB intake | RB dedinho | Y hive | X captura");
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
