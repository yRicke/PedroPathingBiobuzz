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
import org.firstinspires.ftc.teamcode.subsystems.FlowerCollect.FlowerCollect;
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
import java.util.Arrays;
import java.util.List;
import java.util.OptionalDouble;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;

import static org.junit.Assert.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

public class AutonomousSouthOutsideV3Test {
    private final AtomicLong clock = new AtomicLong();
    private final Localizer localizer = mock(Localizer.class, CALLS_REAL_METHODS);
    private final Drivetrain drivetrain = mock(Drivetrain.class);
    private final Intake intake = mock(Intake.class);
    private final Flywheel flywheel = mock(Flywheel.class);
    private final Transfer transfer = mock(Transfer.class);
    private final Trigger trigger = mock(Trigger.class);
    private final Limelight limelight = mock(Limelight.class);
    private final FlowerCollect flowerCollect = mock(FlowerCollect.class);
    private final AutonomousSouthOutsideV3 auto = new AutonomousSouthOutsideV3();
    private final List<Path> followed = new ArrayList<>();
    private final List<Long> shotStarts = new ArrayList<>();
    private final List<Long> shotEnds = new ArrayList<>();
    private final List<Integer> shotPathCounts = new ArrayList<>();
    private final List<Integer> shotTags = new ArrayList<>();
    private Pose pose = new Pose(57.8, 9, Math.PI / 2);
    private Velocity velocity = Velocity.zero();
    private Follower follower;
    private boolean feeding, transferring, flowerOpen, visible = true;
    private int selectedTag = 42;
    private long openedAt, closedAt;
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
            assertFalse("Nao deve iniciar um trecho durante um disparo", feeding || transferring);
            int nextIndex = followed.size();
            if (nextIndex == 6) {
                assertTrue("O coletor deve estar aberto antes de coletar", flowerOpen);
                assertTrue(millis(clock.get() - openedAt) >= 500);
            }
            if (nextIndex == 8) {
                assertFalse("O coletor deve estar recolhido antes de retornar ao lancamento", flowerOpen);
                assertTrue(millis(clock.get() - closedAt) >= 500);
            }
            followed.add(inv.getArgument(0));
            return inv.callRealMethod();
        }).when(follower).follow(any(Path.class));
        doAnswer(inv -> {
            boolean on = inv.getArgument(0);
            if (on && !feeding) {
                assertTrue("O disparo exige hold", follower.holding());
                assertFalse("O disparo exige robo parado", follower.isBusy());
                assertFalse("O coletor deve estar fechado durante os disparos", flowerOpen);
                shotStarts.add(clock.get());
                shotPathCounts.add(followed.size());
                shotTags.add(selectedTag);
            } else if (!on && feeding) {
                shotEnds.add(clock.get());
            }
            feeding = on;
            return null;
        }).when(trigger).run(anyBoolean());
        doAnswer(inv -> { transferring = inv.getArgument(0); return null; })
                .when(transfer).run(anyBoolean());
        doAnswer(inv -> { flowerOpen = true; openedAt = clock.get(); return null; })
                .when(flowerCollect).setOpen();
        doAnswer(inv -> { flowerOpen = false; closedAt = clock.get(); return null; })
                .when(flowerCollect).setClosed();
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
        inject("flowerCollect", flowerCollect);
        inject("nanoClock", (LongSupplier) clock::get);
    }

    @After
    public void tearDown() {
        OpModeStorage.clearAutonomousPose();
    }

    @Test
    public void preservesElevenContinuousPathsCoordinatesAndSuppliedBezierControls() {
        Path[] paths = paths();
        double[][] endpoints = {{57.8, 20}, {31, 10}, {10, 10}, {60, 133}, {60, 122},
                {46.6, 120}, {46.6, 128}, {46.6, 120}, {60, 122}, {60, 133}, {10, 120}};
        assertXY(57.8, 9, paths[0].get(0));
        for (int i = 0; i < paths.length; i++) {
            assertXY(endpoints[i][0], endpoints[i][1], paths[i].endPose());
            if (i > 0) assertPose(paths[i - 1].endPose(), paths[i].get(0));
        }
        assertPose(paths[3].endPose(), paths[9].endPose());
        assertPose(paths[4].endPose(), paths[8].endPose());
        assertPose(paths[5].endPose(), paths[7].endPose());
        assertEquals(paths[0].endPose().y() - paths[0].get(0).y(),
                paths[4].get(0).y() - paths[4].endPose().y(), 1e-6);
        assertXY(30.7122625, 88.3144625, paths[3].get(0.5));
        assertXY(50.3645, 120.30645, paths[10].get(0.5));
        assertAngle(90, paths[0].get(0.5).heading());
        assertAngle(135, paths[1].get(0.5).heading());
        assertAngle(180, paths[2].get(0.5).heading());
        assertAngle(180, paths[3].get(0).heading());
        assertAngle(270, paths[3].endPose().heading());
        assertAngle(270, paths[4].get(0.5).heading());
        assertAngle(90, paths[6].get(0.5).heading());
        assertAngle(90, paths[7].get(0.5).heading());
        assertAngle(270, paths[9].get(0.5).heading());
        assertAngle(270, paths[10].get(0).heading());
        assertAngle(0, paths[10].endPose().heading());
    }

    @Test
    public void followsWholeRouteWithThreeSecondShotsAndThreeAndAHalfSecondFlowerHold() {
        auto.startAutonomous();
        assertFalse(flowerOpen);
        tickMillis(1499);
        assertTrue(followed.isEmpty());
        assertTrue(shotStarts.isEmpty());
        runUntil(() -> feeding, 40);
        assertTrue(followed.isEmpty());
        assertXY(57.8, 9, follower.poseAt(0));
        finishShot(1, 0);

        arrive(auto.chegadinhafrente1(), 2);
        arrive(auto.prepcoletajardim(), 3);
        arrive(auto.coletajardim(), 4);
        pose = auto.lancamento2().endPose();
        velocity = new Velocity(12, 0, 0);
        tick();
        tick();
        assertTrue(follower.isBusy());
        assertEquals(1, shotStarts.size());
        velocity = Velocity.zero();
        runUntil(() -> shotStarts.size() == 2, 40);
        assertXY(60, 133, follower.poseAt(0));
        finishShot(5, 1);

        arrive(auto.chegadinhafrente2(), 6);
        assertTrue(flowerOpen);
        arrive(auto.prepcoletaflower(), 7);
        pose = auto.coletaflower().endPose();
        velocity = new Velocity(12, 0, 0);
        tick();
        tick();
        tickMillis(4000);
        assertEquals("A coleta nao inicia a espera enquanto o robo se move", 7, followed.size());
        velocity = Velocity.zero();
        runUntil(() -> follower.holding() && !follower.isBusy(), 10);
        long collectionStarted = clock.get();
        assertXY(46.6, 128, follower.poseAt(0));
        tickMillis(3499);
        assertEquals(7, followed.size());
        assertTrue(flowerOpen);
        assertFalse(feeding || transferring);
        tickMillis(1);
        assertEquals(8, followed.size());
        assertEquals(3500, millis(clock.get() - collectionStarted));

        pose = auto.posflower().endPose();
        runUntil(() -> !flowerOpen, 10);
        assertEquals(8, followed.size());
        tickMillis(499);
        assertEquals(8, followed.size());
        tickMillis(1);
        assertEquals(9, followed.size());
        arrive(auto.preplancamento3(), 10);
        pose = auto.lancamento3().endPose();
        runUntil(() -> shotStarts.size() == 3, 40);
        assertXY(60, 133, follower.poseAt(0));
        finishShot(11, 2);

        pose = auto.estacionamento().endPose();
        for (int i = 0; i < 5; i++) tick();
        assertTrue(follower.holding());
        assertXY(10, 120, follower.poseAt(0));
        assertEquals(Arrays.asList(0, 4, 10), shotPathCounts);
        assertEquals(Arrays.asList(42, 38, 38), shotTags);
        assertEquals(3, shotStarts.size());
        assertEquals(3, shotEnds.size());
        assertFalse(flowerOpen || feeding || transferring);
        Path[] expected = paths();
        for (int i = 0; i < expected.length; i++) {
            for (double t : new double[] {0, 0.25, 0.5, 0.75, 1}) {
                assertPose(expected[i].get(t), followed.get(i).get(t));
            }
        }
        verify(flywheel, atLeastOnce()).runByDistance(OptionalDouble.of(1.77));
        verify(flywheel, atLeastOnce()).stopMotors();
        verify(intake, atLeastOnce()).offMotor();
    }

    @Test
    public void waitsForVisionBothMotorsAndStableRpmBeforeInitialShot() {
        tx = 6;
        auto.startAutonomous();
        tickMillis(1500);
        for (int i = 0; i < 40; i++) tick();
        assertTrue(shotStarts.isEmpty());
        tx = 0;
        secondRpm = 2000;
        for (int i = 0; i < 30; i++) tick();
        assertTrue(shotStarts.isEmpty());
        secondRpm = 2670;
        firstRpm = 2000;
        for (int i = 0; i < 30; i++) tick();
        assertTrue(shotStarts.isEmpty());
        firstRpm = 2670;
        for (int i = 0; i < 7; i++) tick();
        assertTrue("Prontidao deve persistir por 150 ms", shotStarts.isEmpty());
        runUntil(() -> feeding, 20);
        assertTrue(followed.isEmpty());
    }

    @Test
    public void visionLossStopsFeedingImmediatelyAndRequiresConfirmationAgain() {
        auto.startAutonomous();
        runUntil(() -> feeding, 120);
        visible = false;
        tick();
        assertFalse(feeding || transferring);
        assertTrue(follower.holding());
        assertTrue(followed.isEmpty());
        visible = true;
        tick();
        tick();
        assertFalse(feeding);
        runUntil(() -> feeding, 20);
    }

    @Test
    public void rpmLossStopsFeedingDuringTheShotWindow() {
        auto.startAutonomous();
        runUntil(() -> feeding, 120);
        secondRpm = Double.NaN;
        tick();
        assertFalse(feeding || transferring);
        assertTrue(follower.holding());
        secondRpm = 2670;
        for (int i = 0; i < 7; i++) tick();
        assertFalse(feeding);
        runUntil(() -> feeding, 10);
    }

    @Test
    public void redAllianceUsesOppositeCellInitiallyAndAudienceForBothReturnShots() throws Exception {
        inject("redAlliance", true);
        completeRoute();
        assertEquals(Arrays.asList(30, 34, 34), shotTags);
        assertEquals(Arrays.asList(0, 4, 10), shotPathCounts);
        verify(limelight, atLeastOnce()).run(false, true, false, true);
        verify(limelight, atLeastOnce()).run(true, false, false, true);
    }

    @Test
    public void missingVisionSkipsAllThreeShotsButCompletesFlowerCollectionAndParking() {
        visible = false;
        completeRoute();
        assertTrue(shotStarts.isEmpty());
        assertTrue(shotEnds.isEmpty());
        assertFalse(flowerOpen || feeding || transferring);
        verify(flowerCollect).setOpen();
        verify(flywheel, atLeastOnce()).stopMotors();
    }

    @Test
    public void stopPublishesFreshPoseClosesCollectorAndStopsEverySubsystem() {
        Pose finalPose = new Pose(20, 30, Math.toRadians(23));
        doAnswer(inv -> { pose = finalPose; return null; }).when(localizer).update();
        flowerOpen = true;

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

    private Path[] paths() {
        return new Path[] {auto.chegadinhafrente1(), auto.prepcoletajardim(), auto.coletajardim(),
                auto.lancamento2(), auto.chegadinhafrente2(), auto.prepcoletaflower(), auto.coletaflower(),
                auto.posflower(), auto.preplancamento3(), auto.lancamento3(), auto.estacionamento()};
    }

    private void finishShot(int nextPathCount, int shotIndex) {
        runUntil(() -> followed.size() == nextPathCount, 160);
        assertEquals(3000, millis(shotEnds.get(shotIndex) - shotStarts.get(shotIndex)));
    }

    private void arrive(Path path, int nextPathCount) {
        pose = path.endPose();
        runUntil(() -> followed.size() == nextPathCount, 40);
    }

    private void completeRoute() {
        auto.startAutonomous();
        for (int i = 0; i < 1600; i++) {
            if (follower.following()) pose = follower.currentPath().endPose();
            tick();
            if (followed.size() == 11 && follower.holding() && !follower.isBusy()) {
                tick();
                assertXY(10, 120, follower.poseAt(0));
                return;
            }
        }
        fail("O percurso completo deve terminar");
    }

    private void tick() { tickMillis(20); }

    private void tickMillis(long milliseconds) {
        clock.addAndGet(TimeUnit.MILLISECONDS.toNanos(milliseconds));
        frameTimestamp++;
        auto.updateAutonomous();
    }

    private void runUntil(BooleanSupplier condition, int maxTicks) {
        for (int i = 0; i < maxTicks && !condition.getAsBoolean(); i++) tick();
        assertTrue("A sequencia deve avancar em " + maxTicks + " ciclos", condition.getAsBoolean());
    }

    private void inject(String name, Object value) throws Exception {
        Field field = AutonomousSouthOutsideV3.class.getDeclaredField(name);
        field.setAccessible(true);
        field.set(auto, value);
    }

    private void assertStopped() {
        assertTrue(follower.idle());
        assertFalse(flowerOpen || feeding || transferring);
        verify(drivetrain).stop();
        verify(transfer).run(false);
        verify(trigger).run(false);
        verify(intake).offMotor();
        verify(flywheel).stopMotors();
        verify(flowerCollect).setClosed();
        verify(limelight).stop();
    }

    private static long millis(long nanos) { return TimeUnit.NANOSECONDS.toMillis(nanos); }
    private static void assertXY(double x, double y, Pose actual) {
        assertEquals(x, actual.x(), 1e-6);
        assertEquals(y, actual.y(), 1e-6);
    }
    private static void assertPose(Pose expected, Pose actual) {
        assertXY(expected.x(), expected.y(), actual);
        assertAngle(Math.toDegrees(expected.heading()), actual.heading());
    }
    private static void assertAngle(double degrees, double radians) {
        assertEquals(0, Math.atan2(Math.sin(radians - Math.toRadians(degrees)),
                Math.cos(radians - Math.toRadians(degrees))), 1e-6);
    }
}
