package org.firstinspires.ftc.teamcode.TeleOp;

import org.junit.Test;
import org.junit.Before;
import static org.mockito.Mockito.*;

import static org.junit.Assert.*;

public class FlywheelCalibrationTeleOpTest {
    private FlywheelCalibrationTeleOp controls;

    @Before public void setup() {
        controls = mock(FlywheelCalibrationTeleOp.class, CALLS_REAL_METHODS);
        controls.resetCalibrationControls(2720, 6000);
    }

    @Test
    public void startsOffAndHoldingAOrDpadDoesNotRepeat() {
        assertEquals(0, controls.getCommandedRpm(), 0);
        for (int i = 0; i < 100; i++) controls.updateCalibrationControls(true, false, false, false, true, false);
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
    public void largeStepsRespectBoundsAndStopWinsOverEnable() {
        click(false, false, false, true);
        click(false, false, false, true);
        for (int i = 0; i < 10; i++) click(true, false, false, false);
        assertEquals(6000, controls.getCalibrationRpm(), 0);
        for (int i = 0; i < 10; i++) click(false, true, false, false);
        assertEquals(0, controls.getCalibrationRpm(), 0);
        controls.updateCalibrationControls(false, false, false, false, true, true);
        assertFalse(controls.isFlywheelEnabled());
    }

    private void click(boolean up, boolean down, boolean left, boolean right) {
        controls.updateCalibrationControls(up, down, left, right, false, false);
        controls.updateCalibrationControls(false, false, false, false, false, false);
    }
}
