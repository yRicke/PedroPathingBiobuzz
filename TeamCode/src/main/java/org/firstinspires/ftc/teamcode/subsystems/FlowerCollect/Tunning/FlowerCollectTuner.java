package org.firstinspires.ftc.teamcode.subsystems.FlowerCollect.Tunning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.FlowerCollect.FlowerCollect;

/** Ajuste individual, uma vez por toque, sem acionar outros subsistemas. */
@TeleOp(name = "Flower Collect Tuner", group = "FlowerCollect")
public class FlowerCollectTuner extends OpMode {
    // Milesimos inteiros evitam acumular erro nos ajustes de 0.001.
    private static final int[] STEPS_THOUSANDTHS = {1, 10, 100};
    private FlowerCollect flowerCollect;
    private int positionOneThousandths;
    private int positionTwoThousandths;
    private int stepIndex;
    private boolean servoTwoSelected;
    private boolean previousUp, previousDown, previousLeft, previousRight;

    @Override
    public void init() {
        flowerCollect = new FlowerCollect(hardwareMap, telemetry);
        positionOneThousandths = (int) Math.round(FlowerCollect.SERVO_ONE_OPEN_POSITION * 1000);
        positionTwoThousandths = (int) Math.round(FlowerCollect.SERVO_TWO_OPEN_POSITION * 1000);
        stepIndex = 0;
        servoTwoSelected = false;
        previousUp = previousDown = previousLeft = previousRight = false;
        telemetry.setMsTransmissionInterval(100);
        sendTelemetry();
    }

    @Override
    public void loop() {
        if (gamepad1.a != gamepad1.b) servoTwoSelected = gamepad1.b;

        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        if (left != right) {
            if (left && !previousLeft) stepIndex = Math.max(0, stepIndex - 1);
            if (right && !previousRight) {
                stepIndex = Math.min(STEPS_THOUSANDTHS.length - 1, stepIndex + 1);
            }
        }

        int position = servoTwoSelected ? positionTwoThousandths : positionOneThousandths;
        if (up != down) {
            if (up && !previousUp) position += STEPS_THOUSANDTHS[stepIndex];
            if (down && !previousDown) position -= STEPS_THOUSANDTHS[stepIndex];
        }
        position = Math.max(0, Math.min(1000, position));
        if (servoTwoSelected) {
            positionTwoThousandths = position;
            flowerCollect.setServoTwoPosition(position / 1000.0);
        } else {
            positionOneThousandths = position;
            flowerCollect.setServoOnePosition(position / 1000.0);
        }
        previousUp = up;
        previousDown = down;
        previousLeft = left;
        previousRight = right;
        sendTelemetry();
    }

    private void sendTelemetry() {
        telemetry.addData("Servo selecionado", servoTwoSelected ? 2 : 1);
        telemetry.addData("Passo", "%.3f", STEPS_THOUSANDTHS[stepIndex] / 1000.0);
        flowerCollect.sendTelemetry();
        telemetry.addLine("Controle 1: A = servo 1 | B = servo 2");
        telemetry.addLine("D-pad cima/baixo: posicao | esquerda/direita: passo");
        telemetry.addLine("Um ajuste por toque. Faixa: 0 a 1. Anote as duas posicoes.");
        telemetry.addLine("5-turn: use modo posicional; primeira calibracao sem os bracos.");
        telemetry.update();
    }
}
