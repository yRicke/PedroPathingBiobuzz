package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/** Ajuste manual de um servo posicional por vez, sem instanciar subsistemas. */
@TeleOp(name = "Servo Tuner Universal", group = "Tuning")
public class ServoTuningTeleOp extends OpMode {
    private static final int[] STEPS_THOUSANDTHS = {1, 5, 10, 50, 100};
    private static final PwmControl.PwmRange GOBILDA_RANGE =
            new PwmControl.PwmRange(500, 2500, 20000);

    private List<ServoImplEx> servos;
    private int selectedIndex;
    private int positionThousandths;
    private int stepIndex;
    private boolean enabled;
    private boolean reversed;
    private boolean widePwm;
    private boolean previousUp, previousDown, previousLeft, previousRight;
    private boolean previousA, previousX, previousY, previousLb, previousRb;

    @Override
    public void init() {
        servos = new ArrayList<>(hardwareMap.getAll(ServoImplEx.class));
        servos.sort(Comparator.comparing(this::servoName));
        // Nenhuma posicao e enviada no INIT, inclusive aos servos nao selecionados.
        for (ServoImplEx servo : servos) servo.setPwmDisable();
        selectedIndex = 0;
        resetSelection();
        stepIndex = 0;
        previousUp = previousDown = previousLeft = previousRight = false;
        previousA = previousX = previousY = previousLb = previousRb = false;
        telemetry.setMsTransmissionInterval(100);
        sendTelemetry();
    }

    @Override
    public void init_loop() {
        updateControls(false);
        sendTelemetry();
    }

    @Override
    public void loop() {
        updateControls(true);
        sendTelemetry();
    }

    private void updateControls(boolean canEnable) {
        // B tem prioridade inclusive sobre A e sobre qualquer ajuste simultaneo.
        if (gamepad1.b) {
            if (!servos.isEmpty()) selectedServo().setPwmDisable();
            enabled = false;
        } else if (!servos.isEmpty()) {
            boolean configurationChanged = false;
            if (!enabled) {
                if (gamepad1.left_bumper != gamepad1.right_bumper) {
                    int change = 0;
                    if (gamepad1.left_bumper && !previousLb) change = -1;
                    if (gamepad1.right_bumper && !previousRb) change = 1;
                    if (change != 0 && servos.size() > 1) {
                        selectedIndex = (selectedIndex + change + servos.size()) % servos.size();
                        resetSelection();
                        configurationChanged = true;
                    }
                }
                if (gamepad1.x && !previousX) {
                    widePwm = !widePwm;
                    configurationChanged = true;
                }
                if (gamepad1.y && !previousY) {
                    reversed = !reversed;
                    configurationChanged = true;
                }
            }

            if (gamepad1.dpad_left != gamepad1.dpad_right) {
                if (gamepad1.dpad_left && !previousLeft) stepIndex = Math.max(0, stepIndex - 1);
                if (gamepad1.dpad_right && !previousRight) {
                    stepIndex = Math.min(STEPS_THOUSANDTHS.length - 1, stepIndex + 1);
                }
            }

            int previousPosition = positionThousandths;
            if (!configurationChanged && gamepad1.dpad_up != gamepad1.dpad_down) {
                if (gamepad1.dpad_up && !previousUp) {
                    positionThousandths += STEPS_THOUSANDTHS[stepIndex];
                }
                if (gamepad1.dpad_down && !previousDown) {
                    positionThousandths -= STEPS_THOUSANDTHS[stepIndex];
                }
                positionThousandths = Math.max(0, Math.min(1000, positionThousandths));
            }

            if (enabled && positionThousandths != previousPosition) {
                selectedServo().setPosition(positionThousandths / 1000.0);
            }
            if (canEnable && !configurationChanged && !enabled && gamepad1.a && !previousA) {
                ServoImplEx servo = selectedServo();
                servo.setDirection(reversed ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
                servo.scaleRange(0, 1);
                servo.setPwmRange(selectedRange());
                servo.setPosition(positionThousandths / 1000.0);
                servo.setPwmEnable();
                enabled = true;
            }
        }

        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;
        previousLeft = gamepad1.dpad_left;
        previousRight = gamepad1.dpad_right;
        previousA = gamepad1.a;
        previousX = gamepad1.x;
        previousY = gamepad1.y;
        previousLb = gamepad1.left_bumper;
        previousRb = gamepad1.right_bumper;
    }

    private void resetSelection() {
        positionThousandths = 500;
        reversed = widePwm = enabled = false;
    }

    private ServoImplEx selectedServo() {
        return servos.get(selectedIndex);
    }

    private PwmControl.PwmRange selectedRange() {
        return widePwm ? GOBILDA_RANGE : PwmControl.PwmRange.defaultRange;
    }

    private String servoName(ServoImplEx servo) {
        List<String> names = new ArrayList<>(hardwareMap.getNamesOf(servo));
        Collections.sort(names);
        return String.join(" / ", names);
    }

    private void sendTelemetry() {
        if (servos.isEmpty()) {
            telemetry.addLine("Nenhum servo posicional compativel encontrado.");
            telemetry.addLine("Configure a porta como Servo no Hub e reinicie o INIT.");
            telemetry.addLine("Este tuner requer ServoImplEx (Hub REV); CRServo nao e listado.");
        } else {
            ServoImplEx servo = selectedServo();
            PwmControl.PwmRange range = selectedRange();
            double position = positionThousandths / 1000.0;
            double signalPosition = reversed ? 1 - position : position;
            double pulseUs = range.usPulseLower
                    + signalPosition * (range.usPulseUpper - range.usPulseLower);
            telemetry.addData("Servo selecionado", "%d/%d: %s",
                    selectedIndex + 1, servos.size(), servoName(servo));
            telemetry.addData("Conexao", servo.getConnectionInfo());
            telemetry.addData("Porta no Hub", servo.getPortNumber());
            telemetry.addData("PWM", enabled ? "LIGADO" : "DESLIGADO - A apos Play para aplicar");
            telemetry.addData("Posicao alvo (0 a 1)", "%.3f", position);
            telemetry.addData("Passo", "%.3f", STEPS_THOUSANDTHS[stepIndex] / 1000.0);
            telemetry.addData("Sentido", reversed ? "REVERSE" : "FORWARD");
            telemetry.addData("Perfil PWM", widePwm ? "goBILDA 5-turn" : "Padrao SDK");
            telemetry.addData("Faixa PWM (us)", "%.0f a %.0f", range.usPulseLower, range.usPulseUpper);
            telemetry.addData("Pulso alvo (us)", "%.1f%s", pulseUs, enabled ? "" : " (nao enviado)");
            telemetry.addLine("G1: A aplica/liga | B desliga PWM");
            telemetry.addLine("D-pad cima/baixo: posicao | esquerda/direita: passo");
            telemetry.addLine("Com PWM desligado: LB/RB servo | X faixa PWM | Y sentido");
            telemetry.addLine("Trocar servo reinicia alvo em 0.500, FORWARD e PWM padrao.");
            telemetry.addLine("Um ajuste por toque. Anote nome, posicao, sentido e faixa PWM.");
            telemetry.addLine("Alvo nao e leitura fisica. Nao ha gravacao automatica.");
            telemetry.addLine("5-turn: modo posicional; teste inicialmente sem o mecanismo.");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (servos != null) {
            for (ServoImplEx servo : servos) servo.setPwmDisable();
        }
        enabled = false;
    }
}
