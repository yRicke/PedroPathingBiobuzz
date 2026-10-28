package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced;

import org.junit.Test;

import static org.junit.Assert.*;

public class FlywheelCalibrationControlsTest {
    private final FlywheelCalibrationControls controls = new FlywheelCalibrationControls(2720, 6000);

    @Test
    public void startsOffAndHoldingAOrDpadDoesNotRepeat() {
        assertEquals(0, controls.getCommandedRpm(), 0);
        for (int i = 0; i < 100; i++) controls.update(true, false, false, false, true, false);
        assertTrue(controls.isEnabled());
        assertEquals(2730, controls.getCommandedRpm(), 0);
    }

    @Test
    public void selectsAllFiveStepsAndTenthsRemainExactAcrossManyClicks() {
        click(false, false, true, false);
        assertEquals(1, controls.getStepRpm(), 0);
        click(false, false, true, false);
        assertEquals(0.1, controls.getStepRpm(), 0);
        for (int i = 0; i < 100; i++) click(true, false, false, false);
        assertEquals(2730, controls.getTargetRpm(), 0);
        for (double step : new double[] {1, 10, 100, 1000}) {
            click(false, false, false, true);
            assertEquals(step, controls.getStepRpm(), 0);
        }
        click(false, false, false, true);
        assertEquals(1000, controls.getStepRpm(), 0);
    }

    @Test
    public void largeStepsRespectBoundsAndStopWinsOverEnable() {
        click(false, false, false, true);
        click(false, false, false, true);
        for (int i = 0; i < 10; i++) click(true, false, false, false);
        assertEquals(6000, controls.getTargetRpm(), 0);
        for (int i = 0; i < 10; i++) click(false, true, false, false);
        assertEquals(0, controls.getTargetRpm(), 0);
        controls.update(false, false, false, false, true, true);
        assertFalse(controls.isEnabled());
    }

    private void click(boolean up, boolean down, boolean left, boolean right) {
        controls.update(up, down, left, right, false, false);
        controls.update(false, false, false, false, false, false);
    }
}
