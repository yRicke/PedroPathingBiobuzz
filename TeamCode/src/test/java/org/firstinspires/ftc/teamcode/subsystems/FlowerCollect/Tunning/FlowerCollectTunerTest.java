package org.firstinspires.ftc.teamcode.subsystems.FlowerCollect.Tunning;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Before;
import org.junit.Test;

import static org.mockito.Mockito.*;

public class FlowerCollectTunerTest {
    private FlowerCollectTuner tuner;
    private Servo one, two;

    @Before public void setup() {
        one = mock(Servo.class);
        two = mock(Servo.class);
        tuner = mock(FlowerCollectTuner.class, CALLS_REAL_METHODS);
        tuner.hardwareMap = mock(HardwareMap.class);
        tuner.telemetry = mock(Telemetry.class);
        tuner.gamepad1 = mock(Gamepad.class);
        when(tuner.hardwareMap.get(Servo.class, "flowerCollect1")).thenReturn(one);
        when(tuner.hardwareMap.get(Servo.class, "flowerCollect2")).thenReturn(two);
        tuner.init();
        clearInvocations(one, two);
    }

    @Test public void holdingUpOnlyMovesOneSmallStep() {
        tuner.gamepad1.dpad_up = true;
        for (int i = 0; i < 20; i++) tuner.loop();
        verify(one, times(20)).setPosition(0.001);
        verifyNoInteractions(two);
    }

    @Test public void selectsAllStepsAndClampsStepSelectionAtBothEnds() {
        click(false, false, true, false);
        click(true, false, false, false);
        assertCommand(0.001);
        click(false, false, false, true);
        click(true, false, false, false);
        assertCommand(0.011);
        click(false, false, false, true);
        click(false, false, false, true);
        click(true, false, false, false);
        assertCommand(0.111);
        click(false, false, true, false);
        click(false, true, false, false);
        assertCommand(0.101);
        click(false, false, true, false);
        click(false, true, false, false);
        assertCommand(0.100);
    }

    @Test public void holdingRightOnlyChangesStepOnce() {
        tuner.gamepad1.dpad_right = true;
        for (int i = 0; i < 20; i++) tuner.loop();
        click(false, false, false, false);
        click(true, false, false, false);
        assertCommand(0.01);
    }

    @Test public void manyFineAdjustmentsStayExactAndRespectBothPositionLimits() {
        for (int i = 0; i < 1100; i++) click(true, false, false, false);
        assertCommand(1);
        for (int i = 0; i < 100; i++) click(false, true, false, false);
        assertCommand(0.9);
        for (int i = 0; i < 1000; i++) click(false, true, false, false);
        assertCommand(0);
    }

    @Test public void oppositeButtonsCancelWithoutChangingPositionOrStep() {
        click(true, true, true, true);
        assertCommand(0);
        click(true, false, false, false);
        assertCommand(0.001);
    }

    @Test public void selectionAdjustsEachServoIndependentlyAndRetainsBothPositions() {
        click(true, false, false, false);
        assertCommand(0.001);
        clearInvocations(two);
        tuner.gamepad1.b = true;
        click(true, false, false, false);
        tuner.gamepad1.b = false;
        verify(two, atLeastOnce()).setPosition(0.001);
        verifyNoInteractions(one);
        clearInvocations(two);
        tuner.gamepad1.a = true;
        click(true, false, false, false);
        tuner.gamepad1.a = false;
        assertCommand(0.002);
        verifyNoInteractions(two);
    }

    private void click(boolean up, boolean down, boolean left, boolean right) {
        tuner.gamepad1.dpad_up = up;
        tuner.gamepad1.dpad_down = down;
        tuner.gamepad1.dpad_left = left;
        tuner.gamepad1.dpad_right = right;
        tuner.loop();
        tuner.gamepad1.dpad_up = tuner.gamepad1.dpad_down = false;
        tuner.gamepad1.dpad_left = tuner.gamepad1.dpad_right = false;
        tuner.loop();
    }

    private void assertCommand(double expected) {
        verify(one, atLeastOnce()).setPosition(expected);
        clearInvocations(one, two);
    }
}
