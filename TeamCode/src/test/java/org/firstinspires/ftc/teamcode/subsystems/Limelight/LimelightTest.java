package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.FlywheelRpmCalculator;
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
    public void dpadSelectsHiveAndKeepsSelectionAfterRelease() {
        limelight.run(false, false);
        assertEquals(42, limelight.getObservation().getFirstTagId());

        limelight.run(true, false);
        assertEquals(38, limelight.getObservation().getFirstTagId());
        limelight.run(false, false);
        assertEquals(38, limelight.getObservation().getFirstTagId());
        limelight.run(true, true);
        assertEquals(38, limelight.getObservation().getFirstTagId());

        limelight.run(false, true);
        assertEquals(42, limelight.getObservation().getFirstTagId());
    }

    @Test
    public void readsEstimatedCenterFromSelectedHiveIgnoringCrosshair() {
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
        assertEquals(2 / Math.cos(Math.toRadians(7)),
                inverted.getDistanceMeters().getAsDouble(), 1e-9);
    }

    @Test
    public void otherTagsAllowSearchWithoutBecomingTheTarget() {
        setTags(tag(39, -3));

        limelight.run(false, true);

        assertTrue(limelight.getObservation().isVisionReady());
        assertFalse(limelight.getObservation().hasTarget());
        assertFalse(limelight.getDistanceMeters().isPresent());
    }

    @Test
    public void recentImageWithoutDetectionsAllowsSearch() {
        when(frame.isValid()).thenReturn(false);

        limelight.run(false, true);

        assertTrue(limelight.getObservation().isVisionReady());
        assertFalse(limelight.getObservation().hasTarget());
        assertFalse(limelight.getDistanceMeters().isPresent());
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
        assertFalse(limelight.getDistanceMeters().isPresent());

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
        assertFalse(limelight.getDistanceMeters().isPresent());
    }

    @Test
    public void frozenCameraFramesBecomeUnavailableEvenWithFreshPolling() {
        limelight.run(false, true);
        assertTrue(limelight.getObservation().hasTarget());

        clock.set(151_000_000L);
        limelight.run(false, true);
        assertFalse(limelight.getObservation().isVisionReady());
        assertFalse(limelight.getDistanceMeters().isPresent());

        when(frame.getTimestamp()).thenReturn(2.0);
        limelight.run(false, true);
        assertTrue(limelight.getObservation().hasTarget());
    }

    @Test
    public void missingCameraOrResultIsUnavailable() {
        Limelight absent = new Limelight((Limelight3A) null, null, false, 0);
        absent.run(true, false);
        assertFalse(absent.getObservation().isVisionReady());
        assertEquals(38, absent.getObservation().getFirstTagId());
        assertFalse(absent.getDistanceMeters().isPresent());

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
    public void nonFiniteCenterPositionCannotBecomeAnAimTarget() {
        setTags(tag(43, Double.NaN));
        limelight.run(false, true);
        assertFalse(limelight.getObservation().hasTarget());

        setTags(tag(43, Double.POSITIVE_INFINITY));
        limelight.run(false, true);
        assertFalse(limelight.getObservation().hasTarget());
        assertFalse(limelight.getDistanceMeters().isPresent());
    }

    @Test
    public void anyTagFromSelectedHiveReportsDistanceToItsCenter() {
        for (int id = 38; id <= 45; id++) {
            setTags(tag(id, 7));
            limelight.run(id < 42, id >= 42);

            assertTrue("Tag " + id, limelight.getDistanceMeters().isPresent());
            assertEquals(2 / Math.cos(Math.toRadians(7)),
                    limelight.getDistanceMeters().getAsDouble(), 1e-9);
            assertEquals(7, limelight.getObservation().getTxDegrees(), 1e-9);
            assertEquals(1, limelight.getObservation().getTagsUsed());
            assertEquals(id < 42 ? 38 : 42, limelight.getObservation().getFirstTagId());
        }
    }

    @Test
    public void visibleTagChangesKeepSameHiveIdentityAndCenter() {
        setTags(tag(42, 5));
        limelight.run(false, true);
        assertEquals(42, limelight.getObservation().getFirstTagId());

        setTags(tag(45, 5));
        when(frame.getTimestamp()).thenReturn(2.0);
        limelight.run(false, false);

        assertEquals(42, limelight.getObservation().getFirstTagId());
        assertEquals(5, limelight.getObservation().getTxDegrees(), 1e-9);
        assertEquals(2 / Math.cos(Math.toRadians(5)),
                limelight.getDistanceMeters().getAsDouble(), 1e-9);
    }

    @Test
    public void combinesVisibleTagsWithoutIncludingOtherHive() {
        setTags(tag(42, 5), tag(45, 5), tag(38, -20));

        limelight.run(false, true);

        assertEquals(2, limelight.getObservation().getTagsUsed());
        assertEquals(5, limelight.getObservation().getTxDegrees(), 1e-9);
        assertEquals(2 / Math.cos(Math.toRadians(5)),
                limelight.getDistanceMeters().getAsDouble(), 1e-9);
    }

    @Test
    public void visibleTagWithoutFull3dCannotProvideDistanceOrSteer() {
        LLResultTypes.FiducialResult noDepth = mock(LLResultTypes.FiducialResult.class);
        when(noDepth.getFiducialId()).thenReturn(43);
        when(noDepth.getTargetXDegreesNoCrosshair()).thenReturn(7.0);
        setTags(noDepth);

        limelight.run(false, true);

        assertFalse(limelight.getDistanceMeters().isPresent());
        assertFalse(limelight.getObservation().hasTarget());
        assertFalse(limelight.getObservation().isVisionReady());
    }

    @Test
    public void selectingAnUnseenHiveClearsPreviousDistance() {
        setTags(tag(43, 7));
        limelight.run(false, true);
        assertTrue(limelight.getDistanceMeters().isPresent());

        limelight.run(true, false);

        assertFalse(limelight.getDistanceMeters().isPresent());
        assertEquals(38, limelight.getObservation().getFirstTagId());
    }

    @Test
    public void telemetryShowsCentimetersAndClearsValueWhenUnavailable() {
        Telemetry telemetry = mock(Telemetry.class);
        Limelight withTelemetry = new Limelight(camera, telemetry, false, 0, clock::get);
        setTags(tag(43, 0));

        withTelemetry.run(false, true);
        withTelemetry.sendTelemetry();

        verify(telemetry).addData("Distancia ate centro hive (cm)", 200.0);
        verify(telemetry).addData("Tags usadas no centro", 1);

        when(camera.isConnected()).thenReturn(false);
        withTelemetry.run(false, false);
        withTelemetry.sendTelemetry();

        verify(telemetry).addData("Distancia ate centro hive (cm)", "Indisponivel");
        verify(telemetry).addData("Tags usadas no centro", 0);
    }

    @Test
    public void cameraPollingStartsOnceAndStopsWithSubsystem() {
        verify(camera).setPollRateHz(100);
        verify(camera).pipelineSwitch(0);
        verify(camera).start();

        readDistance(1, 0, 2);
        readDistance(2, 25_000_000L, 4);
        assertTrue(limelight.getDistanceMeters().isPresent());

        limelight.stop();

        verify(camera).stop();
        assertFalse(limelight.getObservation().isVisionReady());
        assertFalse(limelight.getDistanceMeters().isPresent());
    }

    @Test
    public void flywheelUsesMeanOfCentralTwentyOfFortyDistances() {
        // Dez baixas, quinze de 2 m, cinco de 6 m e dez altas: centro tem media 3 m.
        for (int i = 0; i < 40; i++) {
            int index = (i * 17) % 40;
            double distance = index < 10 ? 0.5 : index < 25 ? 2 : index < 30 ? 6 : 20;
            readDistance(i + 1, i * 25_000_000L, distance);
        }

        assertEquals(3, limelight.getDistanceMeters().getAsDouble(), 1e-9);
        assertEquals(2700, FlywheelRpmCalculator.calculate(
                limelight.getDistanceMeters(), 2700, 300), 1e-9);
        assertEquals(2, limelight.getObservation().getDistanceMeters().getAsDouble(), 1e-9);
    }

    @Test
    public void distanceFilterUsesAvailableSamplesWhileWarmingUp() {
        readDistance(1, 0, 2);
        assertEquals(2, limelight.getDistanceMeters().getAsDouble(), 1e-9);
        readDistance(2, 25_000_000L, 4);
        assertEquals(3, limelight.getDistanceMeters().getAsDouble(), 1e-9);
        readDistance(3, 50_000_000L, 6);
        assertEquals(4, limelight.getDistanceMeters().getAsDouble(), 1e-9);
        readDistance(4, 75_000_000L, 100);
        assertEquals(5, limelight.getDistanceMeters().getAsDouble(), 1e-9);
    }

    @Test
    public void repeatedFrameDoesNotBiasDistanceFilter() {
        for (int i = 0; i < 4; i++) {
            readDistance(i + 1, i * 25_000_000L, 2);
        }
        readDistance(5, 100_000_000L, 8);

        for (int i = 1; i <= 40; i++) {
            clock.set(100_000_000L + i * 1_000_000L);
            limelight.run(false, false);
        }

        assertEquals(2, limelight.getDistanceMeters().getAsDouble(), 1e-9);
    }

    @Test
    public void limitsDistanceSamplingToOneNewFrameEveryTwentyFiveMilliseconds() {
        readDistance(1, 0, 2);
        for (int i = 1; i < 25; i++) {
            readDistance(i + 1, i * 1_000_000L, 8);
            assertEquals(2, limelight.getDistanceMeters().getAsDouble(), 1e-9);
        }
        readDistance(26, 25_000_000L, 4);
        assertEquals(3, limelight.getDistanceMeters().getAsDouble(), 1e-9);
    }

    @Test
    public void distanceSamplesExpireAfterOneSecond() {
        readDistance(1, 0, 2);
        readDistance(2, 25_000_000L, 4);
        readDistance(3, 1_000_000_000L, 8);

        assertEquals(6, limelight.getDistanceMeters().getAsDouble(), 1e-9);

        readDistance(4, 1_025_000_000L, 10);
        assertEquals(9, limelight.getDistanceMeters().getAsDouble(), 1e-9);
    }

    @Test
    public void losingTargetClearsDistanceHistoryAndUsesBaseRpm() {
        readDistance(1, 0, 2);
        readDistance(2, 25_000_000L, 4);
        when(frame.isValid()).thenReturn(false);
        limelight.run(false, false);
        assertFalse(limelight.getDistanceMeters().isPresent());
        assertEquals(2700, FlywheelRpmCalculator.calculate(
                limelight.getDistanceMeters(), 2700, 300), 1e-9);

        when(frame.isValid()).thenReturn(true);
        readDistance(3, 50_000_000L, 8);
        assertEquals(8, limelight.getDistanceMeters().getAsDouble(), 1e-9);
    }

    @Test
    public void switchingHiveDoesNotMixDistanceSamplesEvenInSameFrame() {
        setTags(tag(43, 0, 2), tag(39, 0, 8));
        limelight.run(false, true);
        assertEquals(2, limelight.getDistanceMeters().getAsDouble(), 1e-9);

        limelight.run(true, false);

        assertEquals(8, limelight.getDistanceMeters().getAsDouble(), 1e-9);
    }

    @Test
    public void disconnectClearsDistanceHistoryBeforeReacquisition() {
        readDistance(1, 0, 2);
        readDistance(2, 25_000_000L, 4);
        when(camera.isConnected()).thenReturn(false);
        limelight.run(false, false);
        assertFalse(limelight.getDistanceMeters().isPresent());

        when(camera.isConnected()).thenReturn(true);
        readDistance(3, 50_000_000L, 8);
        assertEquals(8, limelight.getDistanceMeters().getAsDouble(), 1e-9);
    }

    @Test
    public void frozenFramesClearDistanceHistoryBeforeReacquisition() {
        readDistance(1, 0, 2);
        readDistance(2, 25_000_000L, 4);
        clock.set(176_000_000L);
        limelight.run(false, false);
        assertFalse(limelight.getDistanceMeters().isPresent());

        readDistance(3, 177_000_000L, 8);

        assertEquals(8, limelight.getDistanceMeters().getAsDouble(), 1e-9);
    }

    @Test
    public void telemetryDistinguishesFilteredAndInstantaneousDistances() {
        Telemetry telemetry = mock(Telemetry.class);
        limelight = new Limelight(camera, telemetry, false, 0, clock::get);
        readDistance(1, 0, 2);
        readDistance(2, 25_000_000L, 4);

        limelight.sendTelemetry();

        verify(telemetry).addData("Distancia ate centro hive (cm)", 300.0);
        verify(telemetry).addData("Distancia instantanea centro hive (cm)", 400.0);
        verify(telemetry).addData("Amostras distancia flywheel", 2);
    }

    private void readDistance(double timestamp, long nanos, double distance) {
        clock.set(nanos);
        when(frame.getTimestamp()).thenReturn(timestamp);
        setTags(tag(43, 0, distance));
        limelight.run(false, true);
    }

    private void setTags(LLResultTypes.FiducialResult... tags) {
        when(frame.getFiducialResults()).thenReturn(Arrays.asList(tags));
    }

    private LLResultTypes.FiducialResult tag(int id, double tx) {
        return tag(id, tx, 2);
    }

    private LLResultTypes.FiducialResult tag(int id, double tx, double depth) {
        LLResultTypes.FiducialResult tag = mock(LLResultTypes.FiducialResult.class);
        when(tag.getFiducialId()).thenReturn(id);
        when(tag.getTargetXDegreesNoCrosshair()).thenReturn(tx);
        when(tag.getTargetXDegrees()).thenReturn(20.0);
        double[] offsets = {0.1651, 0.06985, -0.06985, -0.1651};
        double centerX = depth * Math.tan(Math.toRadians(tx));
        double offset = offsets[(id - 38) % 4];
        when(tag.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                new Position(DistanceUnit.METER, centerX - offset, 0, depth, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0)));
        when(tag.getTargetArea()).thenReturn(1.0);
        return tag;
    }
}
