package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

public class LimelightTest {
    private Limelight3A camera;
    private LLResult frame;
    private Limelight limelight;
    private final AtomicLong clock = new AtomicLong();

    @Before
    public void setUp() {
        camera = mock(Limelight3A.class);
        frame = mock(LLResult.class);
        when(camera.isConnected()).thenReturn(true);
        when(camera.isRunning()).thenReturn(true);
        when(camera.getLatestResult()).thenReturn(frame);
        when(frame.getTimestamp()).thenReturn(1.0);
        when(frame.isValid()).thenReturn(true);
        setTags(tag(43, 7), tag(39, -3));
        limelight = new Limelight(camera, null, false, 0, clock::get);
    }

    @Test
    public void dpadSelectsRequestedIdAndKeepsSelectionAfterRelease() {
        limelight.run(false, false);
        assertEquals(43, limelight.getObservation().getTagId());

        limelight.run(true, false);
        assertEquals(39, limelight.getObservation().getTagId());
        limelight.run(false, false);
        assertEquals(39, limelight.getObservation().getTagId());
        limelight.run(true, true);
        assertEquals(39, limelight.getObservation().getTagId());

        limelight.run(false, true);
        assertEquals(43, limelight.getObservation().getTagId());
    }

    @Test
    public void readsHorizontalOffsetFromSelectedTagIgnoringCrosshair() {
        when(frame.getTx()).thenReturn(25.0);

        limelight.run(false, true);
        assertTrue(limelight.getObservation().hasTarget());
        assertEquals(7, limelight.getObservation().getTxDegrees(), 1e-9);

        limelight.run(true, false);
        assertEquals(-3, limelight.getObservation().getTxDegrees(), 1e-9);
    }

    @Test
    public void upsideDownMountInvertsHorizontalOffset() {
        Limelight inverted = new Limelight(camera, null, true, 0);

        inverted.run(false, true);

        assertEquals(-7, inverted.getObservation().getTxDegrees(), 1e-9);
    }

    @Test
    public void otherTagsAllowSearchWithoutBecomingTheTarget() {
        setTags(tag(39, -3));

        limelight.run(false, true);

        assertTrue(limelight.getObservation().isVisionReady());
        assertFalse(limelight.getObservation().hasTarget());
    }

    @Test
    public void recentImageWithoutDetectionsAllowsSearch() {
        when(frame.isValid()).thenReturn(false);

        limelight.run(false, true);

        assertTrue(limelight.getObservation().isVisionReady());
        assertFalse(limelight.getObservation().hasTarget());
    }

    @Test
    public void staleOrFutureDatedImagesCannotSteerOrSearch() {
        when(frame.getStaleness()).thenReturn(150L);
        limelight.run(false, true);
        assertTrue(limelight.getObservation().hasTarget());

        when(frame.getStaleness()).thenReturn(151L);
        limelight.run(false, true);
        assertFalse(limelight.getObservation().isVisionReady());
        assertFalse(limelight.getObservation().hasTarget());

        when(frame.getStaleness()).thenReturn(-1L);
        limelight.run(false, true);
        assertFalse(limelight.getObservation().isVisionReady());
    }

    @Test
    public void disconnectClearsPreviousDetection() {
        limelight.run(false, true);
        assertTrue(limelight.getObservation().hasTarget());

        when(camera.isConnected()).thenReturn(false);
        limelight.run(false, true);

        assertFalse(limelight.getObservation().isVisionReady());
        assertFalse(limelight.getObservation().hasTarget());
    }

    @Test
    public void frozenCameraFramesBecomeUnavailableEvenWithFreshPolling() {
        limelight.run(false, true);
        assertTrue(limelight.getObservation().hasTarget());

        clock.set(151_000_000L);
        limelight.run(false, true);
        assertFalse(limelight.getObservation().isVisionReady());

        when(frame.getTimestamp()).thenReturn(2.0);
        limelight.run(false, true);
        assertTrue(limelight.getObservation().hasTarget());
    }

    @Test
    public void missingCameraOrResultIsUnavailable() {
        Limelight absent = new Limelight((Limelight3A) null, null, false, 0);
        absent.run(true, false);
        assertFalse(absent.getObservation().isVisionReady());
        assertEquals(39, absent.getObservation().getTagId());

        when(camera.getLatestResult()).thenReturn(null);
        limelight.run(false, true);
        assertFalse(limelight.getObservation().isVisionReady());
    }

    @Test
    public void wrongPipelineOrInvalidTimestampIsUnavailable() {
        when(frame.getPipelineIndex()).thenReturn(1);
        limelight.run(false, true);
        assertFalse(limelight.getObservation().isVisionReady());

        when(frame.getPipelineIndex()).thenReturn(0);
        when(frame.getTimestamp()).thenReturn(Double.NaN);
        limelight.run(false, true);
        assertFalse(limelight.getObservation().isVisionReady());
    }

    @Test
    public void nonFiniteAnglesCannotBecomeAnAimTarget() {
        setTags(tag(43, Double.NaN));
        limelight.run(false, true);
        assertFalse(limelight.getObservation().hasTarget());

        setTags(tag(43, Double.POSITIVE_INFINITY));
        limelight.run(false, true);
        assertFalse(limelight.getObservation().hasTarget());
    }

    @Test
    public void cameraPollingStartsOnceAndStopsWithSubsystem() {
        verify(camera).setPollRateHz(100);
        verify(camera).pipelineSwitch(0);
        verify(camera).start();

        limelight.stop();

        verify(camera).stop();
        assertFalse(limelight.getObservation().isVisionReady());
    }

    private void setTags(LLResultTypes.FiducialResult... tags) {
        when(frame.getFiducialResults()).thenReturn(Arrays.asList(tags));
    }

    private LLResultTypes.FiducialResult tag(int id, double tx) {
        LLResultTypes.FiducialResult tag = mock(LLResultTypes.FiducialResult.class);
        when(tag.getFiducialId()).thenReturn(id);
        when(tag.getTargetXDegreesNoCrosshair()).thenReturn(tx);
        when(tag.getTargetXDegrees()).thenReturn(20.0);
        return tag;
    }
}
