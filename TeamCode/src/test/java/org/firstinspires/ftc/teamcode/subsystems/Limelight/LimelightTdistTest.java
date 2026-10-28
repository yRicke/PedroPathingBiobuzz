package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.json.JSONArray;
import org.json.JSONObject;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

/** Usa o LLResult real do SDK para verificar o acesso ao JSON, nao apenas um mock. */
public class LimelightTdistTest {
    private final AtomicLong clock = new AtomicLong();
    private Limelight3A camera;
    private Limelight limelight;

    @Before
    public void setup() {
        camera = mock(Limelight3A.class);
        when(camera.isConnected()).thenReturn(true);
        when(camera.isRunning()).thenReturn(true);
        limelight = new Limelight(camera, null, true, 8, clock::get);
    }

    @Test
    public void readsRealSdkJsonTdistEvenWhenLegacyCenterHasNoPose() throws Exception {
        read(1, 0, 1.75, 42);
        assertEquals(1.75, limelight.getRawTdistMeters().getAsDouble(), 1e-9);
        assertEquals(1.75, limelight.getTdistMeters().getAsDouble(), 1e-9);
        assertFalse(limelight.getDistanceMeters().isPresent());
    }

    @Test
    public void missingZeroOrMalformedTdistNeverFallsBackToTheOldDistance() throws Exception {
        read(1, 0, 2, 42);
        JSONObject json = json(2, 0, 42);
        json.remove("tdist");
        when(camera.getLatestResult()).thenReturn(new Frame(json));
        limelight.run(false, false);
        assertFalse(limelight.getTdistMeters().isPresent());
        assertEquals(0, limelight.getTdistSampleCount());
        read(3, 50_000_000, 0, 42);
        assertFalse(limelight.getTdistMeters().isPresent());
        json = json(4, 2, 42).put("tdist", "invalido");
        when(camera.getLatestResult()).thenReturn(new Frame(json));
        limelight.run(false, false);
        assertFalse(limelight.getTdistMeters().isPresent());
    }

    @Test
    public void rejectsMixedHivesAndDoesNotMixHistoriesAfterSwitching() throws Exception {
        read(1, 0, 2, 42);
        read(2, 25_000_000, 8, 42, 38);
        assertFalse(limelight.getTdistMeters().isPresent());
        when(camera.getLatestResult()).thenReturn(new Frame(json(3, 4, 38)));
        clock.set(50_000_000);
        limelight.run(true, false);
        assertEquals(4, limelight.getTdistMeters().getAsDouble(), 1e-9);
        assertEquals(1, limelight.getTdistSampleCount());
    }

    @Test
    public void repeatedFramesDoNotFillFortySamplesAndFrozenFramesClearDistance() throws Exception {
        read(1, 0, 2, 42);
        for (int i = 0; i < 100; i++) {
            clock.set(i * 1_000_000L);
            limelight.run(false, false);
        }
        assertEquals(1, limelight.getTdistSampleCount());
        clock.set(151_000_000);
        limelight.run(false, false);
        assertFalse(limelight.getTdistMeters().isPresent());
        assertFalse(limelight.getRawTdistMeters().isPresent());
    }

    @Test
    public void averagesCentralTwentyOfFortyUniqueTdistValues() throws Exception {
        for (int i = 0; i < 40; i++) {
            int index = (i * 17) % 40;
            double distance = index < 10 ? 0.5 : index < 25 ? 2 : index < 30 ? 6 : 20;
            read(i + 1, i * 25_000_000L, distance, 42);
        }
        assertEquals(40, limelight.getTdistSampleCount());
        assertEquals(3, limelight.getTdistMeters().getAsDouble(), 1e-9);
    }

    @Test
    public void disconnectWrongPipelineAndStopClearTdist() throws Exception {
        read(1, 0, 2, 42);
        when(camera.isConnected()).thenReturn(false);
        limelight.run(false, false);
        assertFalse(limelight.getTdistMeters().isPresent());
        when(camera.isConnected()).thenReturn(true);
        when(camera.getLatestResult()).thenReturn(new Frame(json(2, 2, 42).put("pID", 7)));
        limelight.run(false, false);
        assertFalse(limelight.getTdistMeters().isPresent());
        read(3, 50_000_000, 2, 42);
        limelight.stop();
        assertFalse(limelight.getTdistMeters().isPresent());
        assertFalse(limelight.getRawTdistMeters().isPresent());
        assertEquals(0, limelight.getTdistSampleCount());
    }

    private void read(double timestamp, long nanos, double distance, int... ids) throws Exception {
        clock.set(nanos);
        when(camera.getLatestResult()).thenReturn(new Frame(json(timestamp, distance, ids)));
        limelight.run(false, false);
    }

    private JSONObject json(double timestamp, double distance, int... ids) throws Exception {
        JSONArray tags = new JSONArray();
        for (int id : ids) tags.put(new JSONObject().put("fID", id));
        return new JSONObject().put("v", 1).put("pID", 8).put("ts", timestamp)
                .put("tdist", distance).put("Fiducial", tags);
    }

    private static final class Frame extends LLResult {
        Frame(JSONObject json) throws Exception { super(json); }
        @Override public long getStaleness() { return 0; }
    }
}
