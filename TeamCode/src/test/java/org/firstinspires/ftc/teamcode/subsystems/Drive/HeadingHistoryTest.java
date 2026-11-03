package org.firstinspires.ftc.teamcode.subsystems.Drive;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class HeadingHistoryTest {
    @Test
    public void interpolatesTheShortArcAcrossPlusAndMinusPi() {
        HeadingHistory history = new HeadingHistory();
        history.record(0, Math.toRadians(179));
        history.record(100, Math.toRadians(-179));

        assertEquals(Math.toRadians(179.5), history.at(25), 1e-9);
        assertEquals(Math.toRadians(-179.5), history.at(75), 1e-9);
        assertTrue(Double.isNaN(history.at(-1)));
        assertTrue(Double.isNaN(history.at(101)));
    }

    @Test
    public void circularHistoryDropsOldSamplesAndResetDropsTheOldReference() {
        HeadingHistory history = new HeadingHistory();
        for (int i = 0; i < 300; i++) history.record(i, i / 1000.0);
        assertTrue(Double.isNaN(history.at(0)));
        assertEquals(0.299, history.at(299), 1e-9);
        history.record(299, 0.5);
        assertEquals(0.5, history.at(299), 1e-9);
        history.record(0, 1);
        assertTrue(Double.isNaN(history.at(299)));
        assertEquals(1, history.at(0), 1e-9);
        history.clear();
        assertTrue(Double.isNaN(history.at(0)));
    }
}
