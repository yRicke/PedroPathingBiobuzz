package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.json.JSONArray;
import org.json.JSONObject;
import org.junit.Before;
import org.junit.Test;
import java.util.concurrent.atomic.AtomicLong;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

/** Exercita o JSON do LLResult real, com doubles apenas no hardware e no relogio. */
public class LimelightTest {
    private Limelight3A camera;
    private Limelight limelight;
    private final AtomicLong clock = new AtomicLong();

    @Before public void setup() {
        camera = mock(Limelight3A.class);
        when(camera.isConnected()).thenReturn(true);
        when(camera.isRunning()).thenReturn(true);
        limelight = new Limelight(camera, null, false, 8, clock::get);
    }

    @Test public void dpadSelectsHiveAndKeepsSelectionAfterRelease() throws Exception {
        frame(json(1, 2, 42, 38));
        limelight.run(false, false, false, false);
        assertEquals(42, limelight.getObservation().getFirstTagId());
        limelight.run(true, false, false, false);
        assertEquals(38, limelight.getObservation().getFirstTagId());
        limelight.run(false, false, false, false);
        limelight.run(true, true, false, false);
        assertEquals(38, limelight.getObservation().getFirstTagId());
        limelight.run(false, true, false, false);
        assertEquals(42, limelight.getObservation().getFirstTagId());
    }

    @Test public void allianceKeepsCellChoiceAndOppositeInputsKeepEachAxis() throws Exception {
        frame(json(1, 2, 42));
        limelight.run(false, false, false, false);
        assertFalse(limelight.isRedAlliance());
        limelight.run(false, false, false, true);
        assertTrue(limelight.isRedAlliance());
        assertEquals(30, limelight.getObservation().getFirstTagId());
        limelight.run(true, false, false, false);
        assertEquals(34, limelight.getObservation().getFirstTagId());
        limelight.run(false, false, true, false);
        assertEquals(38, limelight.getObservation().getFirstTagId());
        limelight.run(true, true, true, true);
        assertEquals(38, limelight.getObservation().getFirstTagId());
        limelight.run(false, true, false, true); // diagonal altera ambos os eixos
        assertEquals(30, limelight.getObservation().getFirstTagId());
        limelight.run(false, false, false, false);
        assertEquals(30, limelight.getObservation().getFirstTagId());
    }

    @Test public void redCellsSupplyPoseDistanceAndAllianceChangeClearsBlueSamples() throws Exception {
        frame(json(1, 2, 42));
        limelight.run(false, false, false, false);
        assertEquals(2, limelight.getDistanceMeters().getAsDouble(), 0);
        // A mesma imagem azul nao pode continuar fornecendo mira ou distancia.
        limelight.run(false, false, false, true);
        assertFalse(limelight.getObservation().hasTarget());
        assertFalse(limelight.getDistanceMeters().isPresent());
        assertEquals(0, limelight.getDistanceSampleCount());
        // Mesmo timestamp: mudanca de alvo exige nova decodificacao.
        frame(json(1, 8, 30, 31, 32, 33));
        limelight.run(false, false, true, false);
        limelight.run(false, false, false, true);
        assertEquals(8, limelight.getDistanceMeters().getAsDouble(), 0);
        assertEquals(1, limelight.getDistanceSampleCount());
        assertEquals(4, limelight.getObservation().getTagsUsed());
        frame(json(2, 6, 34, 35, 36, 37));
        limelight.run(true, false, false, false);
        assertEquals(6, limelight.getDistanceMeters().getAsDouble(), 0);
        assertEquals(1, limelight.getDistanceSampleCount());
    }

    @Test public void otherCellsDoNotContributeToSelectedRedCellDistance() throws Exception {
        for (int otherId : new int[] {34, 38, 42, 29, 46}) {
            frame(json(1, 3, 30, otherId));
            limelight.run(false, true, false, true);
            assertEquals(30, limelight.getObservation().getFirstTagId());
            assertEquals(1, limelight.getObservation().getTagsUsed());
            assertEquals(3, limelight.getDistanceMeters().getAsDouble(), 1e-9);
            limelight.run(false, false, true, false); // invalida o cache por alvo
        }
    }

    @Test public void redTelemetryShowsAllianceCellAndIds() throws Exception {
        Telemetry telemetry = mock(Telemetry.class);
        limelight = new Limelight(camera, telemetry, false, 8, clock::get);
        frame(json(1, 2, 34));
        limelight.run(true, false, false, true);
        limelight.sendTelemetry();
        verify(telemetry).addData("Alianca", "VERMELHA");
        verify(telemetry).addData("CELL selecionada", "Lado da plateia");
        verify(telemetry).addData("Hive selecionado", "%d-%d", 34, 37);
    }

    @Test public void tdistCannotSupplyDistanceWithoutAValidSelectedCellPose() throws Exception {
        JSONObject json = json(1, 1.75, 42);
        json.put("tdist", 1.75);
        json.getJSONArray("Fiducial").getJSONObject(0).remove("t6t_cs");
        frame(json);
        limelight.run(false, false, false, false);
        assertFalse(limelight.getDistanceMeters().isPresent());
        assertFalse(limelight.getObservation().hasTarget());
    }

    @Test public void readsHeadingFromCenterRatherThanCrosshairAndInvertsMount() throws Exception {
        JSONObject json = json(1, 3, 42).put("tx", 25);
        frame(json);
        limelight.run(false, false, false, false);
        assertTrue(limelight.getObservation().hasTarget());
        assertEquals(0, limelight.getObservation().getTxDegrees(), 1e-9);
        json = json(2, 3, 43);
        json.getJSONArray("Fiducial").getJSONObject(0).put("t6t_cs", new JSONArray(new double[] {0.3 - 0.06985, 0, 2, 0, 0, 0}));
        frame(json);
        Limelight inverted = new Limelight(camera, null, true, 8, clock::get);
        inverted.run(false, false, false, false);
        assertEquals(-Math.toDegrees(Math.atan2(0.3, 2)), inverted.getObservation().getTxDegrees(), 1e-9);
    }

    @Test public void mixedCellsUseOnlySelectedCellPosesEvenWithMisleadingTdist() throws Exception {
        JSONObject json = json(1, 2, 42, 38).put("tdist", 99);
        json.getJSONArray("Fiducial").getJSONObject(1)
                .put("t6t_cs", new JSONArray(new double[] {50, 0, 20, 0, 0, 0}));
        frame(json);
        limelight.run(false, true, false, false);
        assertTrue(limelight.getObservation().hasTarget());
        assertEquals(2, limelight.getDistanceMeters().getAsDouble(), 1e-9);
    }

    @Test public void os2026FramesWithoutTdistSupplyDistanceFromTheCellCenter() throws Exception {
        frame(json(1, 2, 42)); // JSON do OS 2026.0, sem tdist.
        limelight.run(false, false, false, false);
        assertEquals(2, limelight.getDistanceMeters().getAsDouble(), 1e-9);
    }

    @Test public void missingOrInvalidTdistDoesNotAffectPoseDistance() throws Exception {
        for (Object value : new Object[] {JSONObject.NULL, 0, -1, "invalido"}) {
            JSONObject json = json(1 + clock.get(), 2, 42).put("tdist", value);
            frame(json);
            limelight.run(false, false, false, false);
            assertTrue(limelight.getObservation().hasTarget());
            assertEquals(2, limelight.getDistanceMeters().getAsDouble(), 1e-9);
            clock.addAndGet(25_000_000);
        }
    }

    @Test public void staleWrongPipelineAndInvalidTimestampCannotSteerOrSupplyDistance() throws Exception {
        Frame frame = frame(json(1, 2, 42));
        frame.age = 150;
        limelight.run(false, false, false, false);
        assertTrue(limelight.getDistanceMeters().isPresent());
        frame.age = 151;
        limelight.run(false, false, false, false);
        assertUnavailable();
        frame(json(2, 2, 42).put("pID", 7));
        limelight.run(false, false, false, false);
        assertUnavailable();
        frame(json(-1, 2, 42));
        limelight.run(false, false, false, false);
        assertUnavailable();
    }

    @Test public void repeatedTimestampIsDecodedOnlyOnceAndFreezesAfter150Ms() throws Exception {
        Frame first = frame(json(1, 2, 42));
        limelight.run(false, false, false, false);
        for (int i = 1; i <= 100; i++) {
            clock.set(i * 1_000_000L);
            limelight.run(false, false, false, false);
        }
        assertEquals(1, first.tagReads);
        assertEquals(1, limelight.getDistanceSampleCount());
        clock.set(151_000_000);
        limelight.run(false, false, false, false);
        assertUnavailable();
        frame(json(2, 4, 42));
        limelight.run(false, false, false, false);
        assertEquals(4, limelight.getDistanceMeters().getAsDouble(), 0);
    }

    @Test public void selectingAnotherHiveAndDisconnectClearsDistanceHistory() throws Exception {
        frame(json(1, 2, 42));
        limelight.run(false, true, false, false);
        assertEquals(2, limelight.getDistanceMeters().getAsDouble(), 0);
        limelight.run(true, false, false, false);
        assertFalse(limelight.getDistanceMeters().isPresent());
        frame(json(2, 8, 38));
        limelight.run(false, false, false, false);
        assertEquals(8, limelight.getDistanceMeters().getAsDouble(), 0);
        when(camera.isConnected()).thenReturn(false);
        limelight.run(false, false, false, false);
        assertUnavailable();
    }

    @Test public void freshImageWithoutTargetReturnsManualVisionState() throws Exception {
        frame(json(1, 2).put("v", 0));
        limelight.run(false, false, false, false);
        assertTrue(limelight.getObservation().isVisionReady());
        assertFalse(limelight.getObservation().hasTarget());
        assertFalse(limelight.getDistanceMeters().isPresent());
    }

    @Test public void telemetryAndLifecycleExposeCellCenterDistance() throws Exception {
        Telemetry telemetry = mock(Telemetry.class);
        limelight = new Limelight(camera, telemetry, false, 8, clock::get);
        frame(json(1, 2, 42));
        limelight.run(false, false, false, false);
        limelight.sendTelemetry();
        verify(telemetry).addData("Distancia centro filtrada (cm)", "%.3f", 200.0);
        limelight.stop();
        verify(camera).stop();
        assertUnavailable();
        assertEquals(0, limelight.getDistanceSampleCount());
    }

    @Test public void missingCameraAndNullResultAreUnavailable() {
        Limelight missing = new Limelight(null, null, false, 8, clock::get);
        missing.run(false, false, false, false);
        assertFalse(missing.getObservation().isVisionReady());
        when(camera.getLatestResult()).thenReturn(null);
        limelight.run(false, false, false, false);
        assertUnavailable();
    }

    private void assertUnavailable() {
        assertFalse(limelight.getObservation().isVisionReady());
        assertFalse(limelight.getDistanceMeters().isPresent());
        assertFalse(limelight.getRawDistanceMeters().isPresent());
    }
    private Frame frame(JSONObject json) throws Exception {
        Frame frame = new Frame(json);
        when(camera.getLatestResult()).thenReturn(frame);
        return frame;
    }
    private JSONObject json(double timestamp, double distance, int... ids) throws Exception {
        JSONArray tags = new JSONArray();
        double[] offsets = {0.1651, 0.06985, -0.06985, -0.1651};
        for (int id : ids) tags.put(new JSONObject().put("fID", id).put("ta", 1)
                .put("t6t_cs", new JSONArray(new double[] {-offsets[Math.floorMod(id - 30, 4)], 0, distance, 0, 0, 0})));
        return new JSONObject().put("v", 1).put("pID", 8).put("ts", timestamp)
                .put("Fiducial", tags);
    }
    private static final class Frame extends LLResult {
        long age;
        int tagReads;
        Frame(JSONObject json) throws Exception { super(json); }
        @Override public long getStaleness() { return age; }
        @Override public java.util.List<FiducialResult> getFiducialResults() {
            tagReads++;
            return super.getFiducialResults();
        }
    }
}
