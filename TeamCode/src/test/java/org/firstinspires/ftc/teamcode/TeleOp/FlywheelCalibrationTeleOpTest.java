package org.firstinspires.ftc.teamcode.TeleOp;

import org.junit.Test;
import org.junit.Before;
import static org.mockito.Mockito.*;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;

import static org.junit.Assert.*;

public class FlywheelCalibrationTeleOpTest {
    private FlywheelCalibrationTeleOp controls;

    @Before public void setup() {
        controls = mock(FlywheelCalibrationTeleOp.class, CALLS_REAL_METHODS);
        controls.resetCalibrationControls(2720, 0, 6000);
    }

    @Test
    public void calibrationRespectsBaseRpmExtremesAndStillTurnsOff() {
        controls.resetCalibrationControls(Flywheel.BASE_RPM, Flywheel.MIN_RPM, Flywheel.MAX_RPM);
        for (int i = 0; i < 100; i++) click(false, true, false, false);
        assertEquals(Flywheel.MIN_RPM, controls.getCalibrationRpm(), 0);
        for (int i = 0; i < 100; i++) click(true, false, false, false);
        assertEquals(Flywheel.MAX_RPM, controls.getCalibrationRpm(), 0);
        assertEquals(0, controls.getCommandedRpm(), 0);
    }

    @Test
    public void startsOffAndHoldingBOrDpadDoesNotRepeat() {
        assertEquals(0, controls.getCommandedRpm(), 0);
        assertFalse(controls.isIntakeEnabled());
        assertFalse(controls.isTransferEnabled());
        assertFalse(controls.isTriggerEnabled());
        for (int i = 0; i < 100; i++) {
            controls.updateCalibrationControls(true, false, false, false, true, false, false, false);
        }
        assertTrue(controls.isFlywheelEnabled());
        assertEquals(2730, controls.getCommandedRpm(), 0);
    }

    @Test
    public void selectsAllFiveStepsAndTenthsRemainExactAcrossManyClicks() {
        click(false, false, true, false);
        assertEquals(1, controls.getCalibrationStep(), 0);
        click(false, false, true, false);
        assertEquals(0.1, controls.getCalibrationStep(), 0);
        for (int i = 0; i < 100; i++) click(true, false, false, false);
        assertEquals(2730, controls.getCalibrationRpm(), 0);
        for (double step : new double[] {1, 10, 100, 1000}) {
            click(false, false, false, true);
            assertEquals(step, controls.getCalibrationStep(), 0);
        }
        click(false, false, false, true);
        assertEquals(1000, controls.getCalibrationStep(), 0);
    }

    @Test
    public void largeStepsRespectBoundsAndTurningFlywheelOffKeepsTheRpmAdjustment() {
        click(false, false, false, true);
        click(false, false, false, true);
        for (int i = 0; i < 10; i++) click(true, false, false, false);
        assertEquals(6000, controls.getCalibrationRpm(), 0);
        for (int i = 0; i < 10; i++) click(false, true, false, false);
        assertEquals(0, controls.getCalibrationRpm(), 0);
        click(true, false, false, false);
        controls.updateCalibrationControls(false, false, false, false, true, false, false, false);
        assertTrue(controls.isFlywheelEnabled());
        assertEquals(1000, controls.getCommandedRpm(), 0);
        controls.updateCalibrationControls(false, false, false, false, false, false, false, false);
        controls.updateCalibrationControls(false, false, false, false, true, false, false, false);
        assertFalse(controls.isFlywheelEnabled());
        assertEquals(0, controls.getCommandedRpm(), 0);
        assertEquals(1000, controls.getCalibrationRpm(), 0);
    }

    @Test
    public void intakeTogglesWhileTransferAndTriggerFollowRightTrigger() {
        for (int i = 0; i < 100; i++) {
            controls.updateCalibrationControls(false, false, false, false, false, true, false, false);
        }
        assertFalse(controls.isFlywheelEnabled());
        assertTrue(controls.isIntakeEnabled());
        assertFalse(controls.isTransferEnabled());
        assertFalse(controls.isTriggerEnabled());

        controls.updateCalibrationControls(false, false, false, false, false, false, true, false);
        controls.updateCalibrationControls(false, false, false, false, false, false, true, false);
        assertTrue(controls.isTransferEnabled());
        assertTrue(controls.isTriggerEnabled());

        controls.updateCalibrationControls(false, false, false, false, false, false, false, false);
        assertFalse(controls.isTransferEnabled());
        assertFalse(controls.isTriggerEnabled());
        assertTrue(controls.isIntakeEnabled());
        controls.updateCalibrationControls(false, false, false, false, false, true, false, false);
        assertFalse(controls.isIntakeEnabled());
    }

    @Test
    public void leftBumperReversesMomentarilyAndPreservesIntakeToggle() {
        controls.updateCalibrationControls(false, false, false, false, false, true, false, false);
        controls.updateCalibrationControls(false, false, false, false, false, false, false, true);

        assertTrue(controls.isReversing());
        assertTrue(controls.isIntakeEnabled());
        assertFalse(controls.isTriggerEnabled());

        controls.updateCalibrationControls(false, false, false, false, false, false, true, true);
        assertTrue(controls.isReversing());
        assertTrue(controls.isTriggerEnabled());

        controls.updateCalibrationControls(false, false, false, false, false, false, false, false);
        assertFalse(controls.isReversing());
        assertTrue(controls.isIntakeEnabled());
        assertFalse(controls.isTransferEnabled());
        assertFalse(controls.isTriggerEnabled());
    }

    private void click(boolean up, boolean down, boolean left, boolean right) {
        controls.updateCalibrationControls(up, down, left, right, false, false, false, false);
        controls.updateCalibrationControls(false, false, false, false, false, false, false, false);
    }
}
