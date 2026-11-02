package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;

public class OpModeStorageTest {
    @Before
    public void setUp() {
        OpModeStorage.clearAutonomousPose();
    }

    @After
    public void tearDown() {
        OpModeStorage.clearAutonomousPose();
    }

    @Test
    public void standaloneTeleOpHasNoSavedPose() {
        assertNull(OpModeStorage.consumeAutonomousPose());
    }

    @Test
    public void poseIsPassedWithGlobalHeadingAndConsumedOnlyOnce() {
        Pose measuredPose = new Pose(11, 42, Math.toRadians(23));
        OpModeStorage.saveAutonomousPose(measuredPose);

        Pose restored = OpModeStorage.consumeAutonomousPose();

        assertSame(measuredPose, restored);
        assertEquals(23, Math.toDegrees(restored.heading()), 1e-9);
        assertNull(OpModeStorage.consumeAutonomousPose());
    }

    @Test
    public void newAutonomousCancelledBeforePlayInvalidatesPreviousMatchPose() {
        OpModeStorage.saveAutonomousPose(new Pose(100, 100, Math.PI));

        OpModeStorage.clearAutonomousPose();

        assertNull(OpModeStorage.consumeAutonomousPose());
    }

    @Test
    public void latestMeasuredPoseReplacesEarlierPose() {
        OpModeStorage.saveAutonomousPose(new Pose(56, 9, Math.PI / 2));
        Pose stoppedMidPath = new Pose(30, 25, Math.toRadians(23));
        OpModeStorage.saveAutonomousPose(stoppedMidPath);

        assertSame(stoppedMidPath, OpModeStorage.consumeAutonomousPose());
    }
}
