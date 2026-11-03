package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive.PedroDrive;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.AprilTagObservation;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Trigger.Trigger;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

public class AutonomousSouthOutsideV2Test {
    private final AtomicLong clock = new AtomicLong();
    private final Localizer localizer = mock(Localizer.class, CALLS_REAL_METHODS);
    private final Drivetrain drivetrain = mock(Drivetrain.class);
    private final Intake intake = mock(Intake.class);
    private final Flywheel flywheel = mock(Flywheel.class);
    private final Transfer transfer = mock(Transfer.class);
    private final Trigger trigger = mock(Trigger.class);
    private final Limelight limelight = mock(Limelight.class);
    private final AutonomousSouthOutsideV2 auto = new AutonomousSouthOutsideV2();
    private final List<Path> followed = new ArrayList<>();
    private final List<Long> shotStarts = new ArrayList<>();
    private final List<Long> shotEnds = new ArrayList<>();
    private final List<Integer> shotPathCounts = new ArrayList<>();
    private Pose pose = new Pose(57.8428, 8.1749, Math.PI / 2);
    private Velocity velocity = Velocity.zero();
    private Follower follower;
    private boolean feeding, transferring, visible = true;
    private int selectedTag = 42;
    private double frameTimestamp, tx;
    private double firstRpm = 2670, secondRpm = 2670;

    @Before
    public void setUp() throws Exception {
        OpModeStorage.clearAutonomousPose();
        when(localizer.state()).thenAnswer(inv -> MotionState.ofVelocity(pose, velocity));
        doAnswer(inv -> pose).when(localizer).pose();
        doAnswer(inv -> velocity).when(localizer).velocity();
        when(drivetrain.maxScaling(any(), any())).thenReturn(1.0);
        when(drivetrain.interpolateVelocity(anyDouble(), anyDouble(), anyDouble())).thenReturn(1.0);
        follower = spy(new Follower(localizer, drivetrain, new Foresight(Constants.foresightConfig)));
        doAnswer(inv -> { follower.update(0.02); return null; }).when(follower).update();
        doAnswer(inv -> {
            assertFalse("Nao pode iniciar um trecho com o transfer alimentando", transferring);
            followed.add(inv.getArgument(0));
            return inv.callRealMethod();
        }).when(follower).follow(any(Path.class));
        doAnswer(inv -> {
            boolean on = inv.getArgument(0);
            if (on && !feeding) {
                assertTrue(follower.holding());
                assertFalse(follower.isBusy());
                shotStarts.add(clock.get());
                shotPathCounts.add(followed.size());
            } else if (!on && feeding) {
                shotEnds.add(clock.get());
            }
            feeding = on;
            return null;
        }).when(trigger).run(anyBoolean());
        doAnswer(inv -> { transferring = inv.getArgument(0); return null; })
                .when(transfer).run(anyBoolean());
        doAnswer(inv -> {
            boolean audience = inv.getArgument(0), red = inv.getArgument(3);
            selectedTag = red ? (audience ? 34 : 30) : (audience ? 38 : 42);
            return null;
        }).when(limelight).run(anyBoolean(), anyBoolean(), anyBoolean(), anyBoolean());
        when(limelight.getObservation()).thenAnswer(inv -> visible
                ? AprilTagObservation.detected(selectedTag, tx, 2, frameTimestamp)
                : AprilTagObservation.unavailable(selectedTag));
        when(limelight.getDistanceMeters()).thenAnswer(inv ->
                visible ? OptionalDouble.of(1.77) : OptionalDouble.empty());
        when(flywheel.getTargetRPM()).thenReturn(2670.0);
        when(flywheel.getRPM()).thenAnswer(inv -> firstRpm);
        when(flywheel.getSecondMotorRPM()).thenAnswer(inv -> secondRpm);

        inject("follower", follower);
        inject("drive", new PedroDrive(follower, null, Controller.proportional(1)));
        inject("intake", intake);
        inject("flywheel", flywheel);
        inject("transfer", transfer);
        inject("trigger", trigger);
        inject("limelight", limelight);
        inject("nanoClock", (LongSupplier) clock::get);
    }

    @After
    public void tearDown() {
        OpModeStorage.clearAutonomousPose();
    }

    @Test
    public void suppliedEndpointsCurveAndInterpolationsArePreserved() {
        Path[] paths = {auto.path1(), auto.path1_2(), auto.path3(), auto.path4(), auto.path5(), auto.path6()};
        double[][] endpoints = {{57.8922, 18.3733}, {31.0559, 8.6016}, {8.8197, 8.5043},
                {58.9525, 133.3125}, {59.1024, 121.7263}, {9.6898, 121.7273}};
        assertXY(57.8428, 8.1749, paths[0].get(0));
        for (int i = 0; i < paths.length; i++) assertXY(endpoints[i][0], endpoints[i][1], paths[i].endPose());
        assertXY(30.573025, 88.2405125, auto.path4().get(0.5));
        assertAngle(180, auto.path4().get(0).heading());
        assertAngle(270, auto.path4().endPose().heading());
        assertAngle(135, auto.path1_2().get(0.5).heading());
        assertAngle(180, auto.path3().get(0.5).heading());
        assertAngle(89.72246731141259, auto.path1().endPose().heading());
        assertAngle(-89.25875971509723, auto.path5().endPose().heading());
        assertAngle(270, auto.path6().get(0).heading());
        assertAngle(0, auto.path6().endPose().heading());
    }

    @Test
    public void followsAllSixPathsAndShootsOnlyAfterOneAndFourWithStableVisionAndRpm() {
        startAtFirstShot();
        velocity = new Velocity(12, 0, 0);
        tick();
        tick();
        assertTrue(follower.holding());
        assertTrue(follower.isBusy());
        assertTrue(shotStarts.isEmpty());

        velocity = Velocity.zero();
        runUntil(() -> shotStarts.size() == 1, 60);
        assertEquals(Integer.valueOf(1), shotPathCounts.get(0));
        assertXY(57.8922, 18.3733, follower.poseAt(0));
        runUntil(() -> followed.size() == 2, 180);
        assertEquals(3000, millis(shotEnds.get(0) - shotStarts.get(0)));

        arrive(auto.path1_2(), 3);
        arrive(auto.path3(), 4);
        pose = auto.path4().endPose();
        velocity = new Velocity(12, 0, 0);
        tick();
        tick();
        assertEquals(1, shotStarts.size());
        velocity = Velocity.zero();
        runUntil(() -> shotStarts.size() == 2, 60);
        assertEquals(Integer.valueOf(4), shotPathCounts.get(1));
        assertXY(58.9525, 133.3125, follower.poseAt(0));
        runUntil(() -> followed.size() == 5, 180);
        assertEquals(3000, millis(shotEnds.get(1) - shotStarts.get(1)));

        arrive(auto.path5(), 6);
        pose = auto.path6().endPose();
        for (int i = 0; i < 5; i++) tick();
        assertTrue(follower.holding());
        assertXY(9.6898, 121.7273, follower.poseAt(0));
        assertEquals(2, shotStarts.size());
        assertEquals(2, shotEnds.size());
        assertFalse(transferring);
        verify(flywheel, atLeastOnce()).runByDistance(OptionalDouble.of(1.77));
        verify(flywheel, atLeastOnce()).stopMotors();
        verify(intake, atLeastOnce()).offMotor();
        verify(limelight, atLeastOnce()).run(false, true, true, false);
        verify(limelight, atLeastOnce()).run(true, false, true, false);
    }

    @Test
    public void waitsForAlignmentAndBothFlywheelMotorsBeforeFeeding() {
        tx = 6;
        startAtFirstShot();
        for (int i = 0; i < 40; i++) tick();
        assertTrue(shotStarts.isEmpty());
        tx = 0;
        secondRpm = 2000;
        for (int i = 0; i < 40; i++) tick();
        assertTrue(shotStarts.isEmpty());
        secondRpm = 2670;
        for (int i = 0; i < 7; i++) tick();
        assertTrue("A prontidao deve persistir por 150 ms", shotStarts.isEmpty());
        runUntil(() -> feeding, 20);
        assertEquals(1, followed.size());
    }

    @Test
    public void redAllianceUsesRedTagsForBothShotsAndTheSameWaypoints() throws Exception {
        inject("redAlliance", true);
        startAtFirstShot();
        runUntil(() -> feeding, 60);
        assertEquals(30, selectedTag);
        assertXY(57.8922, 18.3733, follower.poseAt(0));
        runUntil(() -> followed.size() == 2, 180);
        arrive(auto.path1_2(), 3);
        arrive(auto.path3(), 4);
        pose = auto.path4().endPose();
        runUntil(() -> shotStarts.size() == 2, 60);
        assertEquals(34, selectedTag);
        assertXY(58.9525, 133.3125, follower.poseAt(0));
        verify(limelight, atLeastOnce()).run(false, true, false, true);
        verify(limelight, atLeastOnce()).run(true, false, false, true);
    }

    @Test
    public void lossOfVisionStopsFeedingImmediatelyWhileEndpointStaysInHold() {
        startAtFirstShot();
        runUntil(() -> feeding, 60);

        visible = false;
        tick();

        assertFalse(feeding);
        assertFalse(transferring);
        assertTrue(follower.holding());
        assertXY(57.8922, 18.3733, follower.poseAt(0));
        visible = true;
        runUntil(() -> feeding, 20);
    }

    @Test
    public void missingVisionSkipsShotAfterTimeoutAndContinuesTheRoute() {
        visible = false;
        startAtFirstShot();
        runUntil(() -> followed.size() == 2, 160);

        assertTrue(shotStarts.isEmpty());
        assertFalse(transferring);
        assertXY(31.0559, 8.6016, followed.get(1).endPose());
    }

    @Test
    public void stopPublishesFreshMeasuredPoseAndStopsEverySubsystem() {
        Pose finalPose = new Pose(20, 30, Math.toRadians(23));
        doAnswer(inv -> { pose = finalPose; return null; }).when(localizer).update();

        auto.finishAutonomous(true);

        assertSame(finalPose, OpModeStorage.consumeAutonomousPose());
        assertStopped();
    }

    @Test
    public void cancelledBeforePlayDoesNotPublishPose() {
        auto.finishAutonomous(false);

        assertNull(OpModeStorage.consumeAutonomousPose());
        verify(localizer, never()).update();
        assertStopped();
    }

    @Test
    public void sensorFailureDuringStopStillStopsEverySubsystem() {
        doThrow(new IllegalStateException("Sensor indisponivel")).when(localizer).update();

        assertThrows(IllegalStateException.class, () -> auto.finishAutonomous(true));

        assertNull(OpModeStorage.consumeAutonomousPose());
        assertStopped();
    }

    private void startAtFirstShot() {
        auto.startAutonomous();
        runUntil(() -> followed.size() == 1, 100);
        pose = auto.path1().endPose();
    }

    private void arrive(Path path, int nextPathCount) {
        pose = path.endPose();
        runUntil(() -> followed.size() == nextPathCount, 10);
    }

    private void tick() {
        clock.addAndGet(20_000_000);
        frameTimestamp++;
        auto.updateAutonomous();
    }

    private void runUntil(BooleanSupplier condition, int maxTicks) {
        for (int i = 0; i < maxTicks && !condition.getAsBoolean(); i++) tick();
        assertTrue("A sequencia deve avancar em " + maxTicks + " ciclos", condition.getAsBoolean());
    }

    private void inject(String name, Object value) throws Exception {
        Field field = AutonomousSouthOutsideV2.class.getDeclaredField(name);
        field.setAccessible(true);
        field.set(auto, value);
    }

    private void assertStopped() {
        assertTrue(follower.idle());
        verify(drivetrain).stop();
        verify(transfer).run(false);
        verify(trigger).run(false);
        verify(intake).offMotor();
        verify(flywheel).stopMotors();
        verify(limelight).stop();
    }

    private static long millis(long nanos) { return TimeUnit.NANOSECONDS.toMillis(nanos); }
    private static void assertXY(double x, double y, Pose actual) {
        assertEquals(x, actual.x(), 1e-6);
        assertEquals(y, actual.y(), 1e-6);
    }
    private static void assertAngle(double degrees, double radians) {
        double difference = radians - Math.toRadians(degrees);
        assertEquals(0, Math.atan2(Math.sin(difference), Math.cos(difference)), 1e-6);
    }
}
