package org.firstinspires.ftc.teamcode.subsystems.Drive;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

public class VisionHeadingTargetTest {
    @Test
    public void filtersAcrossWraparoundWithoutTurningTowardZero() {
        VisionHeadingTarget target = new VisionHeadingTarget();
        target.update(Math.toRadians(179));
        double heading = target.update(Math.toRadians(-179));

        assertEquals(Math.toRadians(179.5), heading, 1e-9);
        assertTrue(target.settled(heading, 0));
    }

    @Test
    public void centralBandHasSeparateEntryAndExitThresholds() {
        VisionHeadingTarget target = new VisionHeadingTarget();
        target.update(0);
        assertFalse(target.settled(Math.toRadians(1.2), 0));
        assertTrue(target.settled(Math.toRadians(0.8), 0));
        assertTrue(target.settled(Math.toRadians(1.2), 0));
        assertFalse(target.settled(Math.toRadians(1.6), 0));
        // Estar atravessando o centro em velocidade ainda exige frenagem.
        assertFalse(target.settled(0, 1));
        assertTrue(target.settled(0, 0));
    }

    @Test
    public void aSpikeHasBoundedEffectAndResetAllowsANewTargetImmediately() {
        VisionHeadingTarget target = new VisionHeadingTarget();
        target.update(0);
        assertEquals(Math.toRadians(1), target.update(Math.toRadians(60)), 1e-9);
        target.reset();
        assertFalse(target.settled(0, 0));
        assertEquals(Math.toRadians(60), target.update(Math.toRadians(60)), 1e-9);
    }
}
