package org.firstinspires.ftc.teamcode.autos;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyBoolean;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.CALLS_REAL_METHODS;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Trigger.Trigger;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.Spy;
import org.mockito.MockitoAnnotations;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

public class AutonomousSouthOutsideTest {
    private Pose pose = new Pose(56, 9, Math.PI / 2);
    private Velocity velocity = Velocity.zero();
    private boolean transferring;
    private final List<Path> followedPaths = new ArrayList<>();
    private final List<Long> shotStarts = new ArrayList<>();
    private final List<Long> shotEnds = new ArrayList<>();
    private final Localizer localizer = mock(Localizer.class, CALLS_REAL_METHODS);
    private final Drivetrain drivetrain = mock(Drivetrain.class);

    @Spy private Follower follower = new Follower(localizer, drivetrain,
            new Foresight(Constants.foresightConfig));
    @Mock private Transfer transfer;
    @Mock private Trigger trigger;
    @InjectMocks private AutonomousSouthOutside auto;
    private AutoCloseable mocks;

    @Before
    public void setUp() {
        Scheduler.reset();
        mocks = MockitoAnnotations.openMocks(this);
        when(localizer.state()).thenAnswer(invocation -> MotionState.ofVelocity(pose, velocity));
        when(drivetrain.maxScaling(any(), any())).thenReturn(1.0);
        when(drivetrain.interpolateVelocity(anyDouble(), anyDouble(), anyDouble())).thenReturn(1.0);
        doAnswer(invocation -> {
            assertFalse("Nao deve iniciar um path durante um disparo", transferring);
            followedPaths.add(invocation.getArgument(0));
            return invocation.callRealMethod();
        }).when(follower).follow(any(Path.class));
        doAnswer(invocation -> {
            boolean enabled = invocation.getArgument(0);
            if (enabled) {
                assertTrue("Todo disparo deve acontecer em hold", follower.holding());
                shotStarts.add(System.nanoTime());
            } else {
                shotEnds.add(System.nanoTime());
            }
            return null;
        }).when(trigger).run(anyBoolean());
        doAnswer(invocation -> {
            transferring = invocation.getArgument(0);
            return null;
        }).when(transfer).run(anyBoolean());
    }

    @After
    public void tearDown() throws Exception {
        Scheduler.reset();
        mocks.close();
    }

    @Test(timeout = 20000)
    public void shootsForFiveSecondsInHoldBeforeForwardAndAfterPath3() throws Exception {
        Command routine = auto.autoRoutine();
        long started = System.nanoTime();
        Scheduler.schedule(routine);

        runUntil(() -> shotStarts.size() == 1, 3000);
        assertTrue("Mantem o aquecimento inicial de 1,5 s",
                elapsedMillis(started) >= 1500);
        assertTrue(followedPaths.isEmpty());
        assertTrue(transferring);
        assertPose(auto.foward().get(0), follower.poseAt(0));

        runUntil(() -> followedPaths.size() == 1, 6000);
        assertShotDuration(0);
        assertPath(auto.foward(), followedPaths.get(0));

        Path[] nextPaths = {auto.path1(), auto.path2(), auto.path3()};
        for (int i = 0; i < nextPaths.length; i++) {
            pose = follower.currentPath().endPose();
            int expectedCount = i + 2;
            runUntil(() -> followedPaths.size() == expectedCount, 1000);
            assertPath(nextPaths[i], follower.currentPath());
            assertEquals(1, shotStarts.size());
            assertFalse(transferring);
        }

        // Chegar ao fim parametrico ainda em movimento nao autoriza o disparo.
        pose = auto.path3().get(0.99);
        velocity = new Velocity(12, 0, 0);
        tick();
        tick();
        assertTrue(follower.atParametricEnd());
        assertTrue(follower.holding());
        assertTrue(follower.isBusy());
        assertEquals(1, shotStarts.size());
        assertFalse(transferring);

        pose = auto.path3().endPose();
        velocity = Velocity.zero();
        runUntil(() -> shotStarts.size() == 2, 1000);
        assertEquals(4, followedPaths.size());
        assertTrue(transferring);
        assertPose(auto.path3().endPose(), follower.poseAt(0));

        runUntil(() -> followedPaths.size() == 5, 6000);
        assertShotDuration(1);
        assertPath(auto.path4(), follower.currentPath());

        pose = auto.path4().endPose();
        runUntil(() -> !Scheduler.isScheduled(routine), 1000);
        assertTrue(follower.holding());
        assertPose(auto.path4().endPose(), follower.poseAt(0));
        assertFalse(transferring);
        assertEquals(2, shotStarts.size());
        assertEquals(2, shotEnds.size());
    }

    private void runUntil(BooleanSupplier condition, long timeoutMs) throws Exception {
        long started = System.nanoTime();
        while (!condition.getAsBoolean() && elapsedMillis(started) < timeoutMs) {
            tick();
            Thread.sleep(1);
        }
        assertTrue("A sequencia nao avancou dentro de " + timeoutMs + " ms", condition.getAsBoolean());
    }

    private void tick() {
        follower.update(0.02);
        Scheduler.execute();
    }

    private void assertShotDuration(int index) {
        long durationMs = TimeUnit.NANOSECONDS.toMillis(shotEnds.get(index) - shotStarts.get(index));
        assertTrue("Disparo terminou antes de 5 s: " + durationMs, durationMs >= 5000);
        assertTrue("Disparo demorou mais de um ciclo alem de 5 s: " + durationMs, durationMs < 5500);
        assertFalse(transferring);
    }

    private static long elapsedMillis(long started) {
        return TimeUnit.NANOSECONDS.toMillis(System.nanoTime() - started);
    }

    private static void assertPath(Path expected, Path actual) {
        for (double t : new double[] {0, 0.25, 0.5, 0.75, 1}) {
            assertPose(expected.get(t), actual.get(t));
        }
    }

    private static void assertPose(Pose expected, Pose actual) {
        assertEquals(expected.x(), actual.x(), 1e-6);
        assertEquals(expected.y(), actual.y(), 1e-6);
        double difference = expected.heading() - actual.heading();
        assertEquals(0, Math.atan2(Math.sin(difference), Math.cos(difference)), 1e-6);
    }
}
