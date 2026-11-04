package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Before;
import org.junit.Test;
import org.mockito.InOrder;

import java.util.Arrays;
import java.util.Collections;

import static org.mockito.Mockito.*;

public class ServoTuningTeleOpTest {
    private ServoTuningTeleOp tuner;
    private ServoImplEx one, two;

    @Before public void setup() {
        one = mock(ServoImplEx.class);
        two = mock(ServoImplEx.class);
        tuner = mock(ServoTuningTeleOp.class, CALLS_REAL_METHODS);
        tuner.hardwareMap = mock(HardwareMap.class);
        tuner.telemetry = mock(Telemetry.class);
        tuner.gamepad1 = mock(Gamepad.class);
        // A ordem de selecao depende do nome, nao da ordem do HardwareMap.
        when(tuner.hardwareMap.getAll(ServoImplEx.class)).thenReturn(Arrays.asList(two, one));
        when(tuner.hardwareMap.getNamesOf(one)).thenReturn(Collections.singleton("flowerCollect1"));
        when(tuner.hardwareMap.getNamesOf(two)).thenReturn(Collections.singleton("flowerCollect2"));
        tuner.init();
    }

    @Test public void initDisablesBothOutputsWithoutSendingAnyPosition() {
        verify(one).setPwmDisable();
        verify(two).setPwmDisable();
        verify(one, never()).setPosition(anyDouble());
        verify(two, never()).setPosition(anyDouble());
        verify(one, never()).setPwmEnable();
        verify(two, never()).setPwmEnable();
    }

    @Test public void holdingAFromInitIntoPlayRequiresReleaseBeforeEnabling() {
        clearInvocations(one, two);
        tuner.gamepad1.a = true;
        tuner.init_loop();
        tuner.loop();
        verifyNoOutputCommands(one);
        verifyNoInteractions(two);
        release();
        enable();
        verify(one).setPosition(0.5);
        verify(one).setPwmEnable();
        verify(two, never()).setPosition(anyDouble());
    }

    @Test public void disabledAdjustmentsAreOnlyAppliedAfterExplicitEnable() {
        clearInvocations(one, two);
        clickUp();
        verifyNoOutputCommands(one);
        verifyNoInteractions(two);
        enable();
        InOrder commands = inOrder(one);
        commands.verify(one).setDirection(Servo.Direction.FORWARD);
        commands.verify(one).scaleRange(0, 1);
        commands.verify(one).setPwmRange(PwmControl.PwmRange.defaultRange);
        commands.verify(one).setPosition(0.501);
        commands.verify(one).setPwmEnable();
        verifyNoInteractions(two);
    }

    @Test public void holdingADoesNotReenableAfterBAndBOverridesAdjustments() {
        enable();
        clearInvocations(one, two);
        tuner.gamepad1.a = true;
        tuner.gamepad1.b = true;
        tuner.gamepad1.dpad_up = true;
        tuner.loop();
        verify(one).setPwmDisable();
        verify(one, never()).setPosition(anyDouble());
        verify(one, never()).setPwmEnable();
        tuner.gamepad1.b = false;
        tuner.loop();
        verify(one, never()).setPwmEnable();
        release();
        enable();
        verify(one).setPosition(0.5);
    }

    @Test public void holdingUpMovesExactlyOneFineStepAndNoOtherServo() {
        enable();
        clearInvocations(one, two);
        tuner.gamepad1.dpad_up = true;
        for (int i = 0; i < 20; i++) tuner.loop();
        verify(one).setPosition(0.501);
        verify(one, times(1)).setPosition(anyDouble());
        verifyNoInteractions(two);
    }

    @Test public void stepsReachBothLimitsAndOppositeInputsCancel() {
        enable();
        // Quatro toques selecionam o passo 0.100; segurar nao avanca novamente.
        for (int i = 0; i < 4; i++) {
            tuner.gamepad1.dpad_right = true;
            tuner.loop();
            release();
        }
        clearInvocations(one, two);
        tuner.gamepad1.dpad_up = tuner.gamepad1.dpad_down = true;
        tuner.loop();
        verify(one, never()).setPosition(anyDouble());
        release();
        for (int i = 0; i < 8; i++) clickUp();
        verify(one).setPosition(1);
        for (int i = 0; i < 12; i++) {
            tuner.gamepad1.dpad_down = true;
            tuner.loop();
            release();
        }
        verify(one).setPosition(0);
        verify(one, never()).setPosition(doubleThat(p -> p < 0 || p > 1));
        verifyNoInteractions(two);
    }

    @Test public void cannotChangeServoDirectionOrRangeWhileEnabled() {
        enable();
        clearInvocations(one, two);
        tuner.gamepad1.right_bumper = tuner.gamepad1.x = tuner.gamepad1.y = true;
        tuner.loop();
        release();
        clickUp();
        verify(one).setPosition(0.501);
        verify(one, never()).setDirection(any());
        verify(one, never()).setPwmRange(any());
        verifyNoInteractions(two);
    }

    @Test public void switchingDisabledServoResetsTargetAndKeepsOldServoOff() {
        enable();
        tuner.gamepad1.b = true;
        tuner.loop();
        release();
        clickUp();
        clearInvocations(one, two);
        tuner.gamepad1.right_bumper = true;
        tuner.loop();
        release();
        enable();
        verify(two).setPosition(0.5);
        verify(two).setPwmEnable();
        verify(one, never()).setPosition(anyDouble());
        verify(one, never()).setPwmEnable();
    }

    @Test public void extendedRangeAndReverseAreAppliedOnlyToSelectedServo() {
        clearInvocations(one, two);
        tuner.gamepad1.x = tuner.gamepad1.y = true;
        tuner.loop();
        release();
        clickUp();
        verifyNoOutputCommands(one);
        verifyNoInteractions(two);
        enable();
        verify(one).setDirection(Servo.Direction.REVERSE);
        verify(one).setPwmRange(new PwmControl.PwmRange(500, 2500, 20000));
        verify(one).setPosition(0.501);
        verify(tuner.telemetry, atLeastOnce()).addData("Pulso alvo (us)", "%.1f%s", 1498.0, "");
        verifyNoInteractions(two);
    }

    @Test public void configurationChangeTogetherWithADoesNotEnableNewConfiguration() {
        clearInvocations(one, two);
        tuner.gamepad1.right_bumper = tuner.gamepad1.a = true;
        tuner.loop();
        verify(one, never()).setPwmEnable();
        verify(two, never()).setPwmEnable();
        release();
        enable();
        verify(two).setPosition(0.5);
    }

    @Test public void stopDisablesEveryServo() {
        enable();
        clearInvocations(one, two);
        tuner.stop();
        verify(one).setPwmDisable();
        verify(two).setPwmDisable();
        verify(one, never()).setPosition(anyDouble());
        verify(two, never()).setPosition(anyDouble());
    }

    @Test public void emptyConfigurationDisplaysInstructionsWithoutCrashing() {
        when(tuner.hardwareMap.getAll(ServoImplEx.class)).thenReturn(Collections.emptyList());
        clearInvocations(one, two);
        tuner.init();
        tuner.gamepad1.a = tuner.gamepad1.right_bumper = true;
        tuner.init_loop();
        tuner.loop();
        tuner.stop();
        verify(tuner.telemetry, atLeastOnce()).addLine("Nenhum servo posicional compativel encontrado.");
        verifyNoInteractions(one, two);
    }

    private void enable() {
        tuner.gamepad1.a = true;
        tuner.loop();
        release();
    }

    private void verifyNoOutputCommands(ServoImplEx servo) {
        // Ler conexao/porta para a telemetria nao deve ser confundido com acionar a saida.
        verify(servo, never()).setPosition(anyDouble());
        verify(servo, never()).setPwmEnable();
        verify(servo, never()).setPwmDisable();
        verify(servo, never()).setDirection(any());
        verify(servo, never()).scaleRange(anyDouble(), anyDouble());
        verify(servo, never()).setPwmRange(any());
    }

    private void clickUp() {
        tuner.gamepad1.dpad_up = true;
        tuner.loop();
        release();
    }

    private void release() {
        tuner.gamepad1.a = tuner.gamepad1.b = tuner.gamepad1.x = tuner.gamepad1.y = false;
        tuner.gamepad1.dpad_up = tuner.gamepad1.dpad_down = false;
        tuner.gamepad1.dpad_left = tuner.gamepad1.dpad_right = false;
        tuner.gamepad1.left_bumper = tuner.gamepad1.right_bumper = false;
        tuner.loop();
    }
}
