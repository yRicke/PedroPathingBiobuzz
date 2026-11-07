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
import com.pedropathing.paths.PathSegment;

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
import org.mockito.InOrder;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.OptionalDouble;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;

import static org.firstinspires.ftc.teamcode.autos.AutonomousConstants.AIM_TIME_MS;
import static org.firstinspires.ftc.teamcode.autos.AutonomousConstants.SHOOT_TIME_MS;
import static org.firstinspires.ftc.teamcode.autos.AutonomousConstants.SPIN_UP_MS;
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
    private final AutoPrincipal auto = new AutoPrincipal();
    private final List<Path> followed = new ArrayList<>();
    private final List<Long> shotStarts = new ArrayList<>();
    private final List<Long> shotEnds = new ArrayList<>();
    private final List<Integer> shotGroupCounts = new ArrayList<>();
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
            if (nextIndex == 2) {
                assertTrue("O coletor deve estar aberto antes de coletar", flowerOpen);
                assertTrue(millis(clock.get() - openedAt) >= 500);
            }
            if (nextIndex == 4) {
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
                shotGroupCounts.add(followed.size());
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
        if (follower != null) follower.stop();
        OpModeStorage.clearAutonomousPose();
    }

    @Test
    public void preservesNinePathsWithDirectCurvedReturnToThirdShot() {
        Path[] paths = paths();
        double[][] endpoints = {{57.8, 20}, {31, 13}, {14, 15}, {60, 147},
                {50.6, 122}, {46.6, 131}, {46.6, 120}, {60, 147}, {10, 120}};
        assertEquals(9, paths.length);
        assertXY(57.8, 9, paths[0].get(0));
        for (int i = 0; i < paths.length; i++) {
            assertXY(endpoints[i][0], endpoints[i][1], paths[i].endPose());
            if (i > 0) assertXY(endpoints[i - 1][0], endpoints[i - 1][1], paths[i].get(0));
        }
        assertXY(60, 147, paths[7].endPose());
        assertEquals(1, paths[7].getSegments().size());
        assertXY(60.38175, 125.4497, paths[7].get(0.5));
        assertXY(30.9622625, 89.5019625, paths[3].get(0.5));
        assertXY(61.56435483870968, 125.58451612903227, paths[4].get(0.5));
        assertXY(50.3645, 123.80645, paths[8].get(0.5));
        assertAngle(90, paths[0].get(0.5).heading());
        assertAngle(135, paths[1].get(0.5).heading());
        assertAngle(180, paths[2].get(0.5).heading());
        assertAngle(180, paths[3].get(0).heading());
        assertAngle(270, paths[3].endPose().heading());
        assertAngle(270, paths[4].get(0).heading());
        assertAngle(90, paths[4].endPose().heading());
        assertAngle(90, paths[5].get(0.5).heading());
        assertAngle(90, paths[6].get(0.5).heading());
        double reverseHeading = Math.atan2(147 - 120, 60 - 46.6) + Math.PI;
        assertAngle(Math.toDegrees(reverseHeading), paths[7].get(0.5).heading());
        double finalReverseHeading = Math.atan2(147 - 117.3994, 60 - 67.4635) + Math.PI;
        assertAngle(Math.toDegrees(finalReverseHeading), paths[7].endPose().heading());
        assertAngle(270, paths[8].get(0).heading());
        assertAngle(0, paths[8].endPose().heading());
    }

    @Test
    public void chainsTransitSegmentsAndKeepsCalibratedShotsAndFlowerHold() {
        auto.startAutonomous();
        assertFalse(flowerOpen);
        tickMillis(SPIN_UP_MS - 1);
        assertTrue(followed.isEmpty());
        assertTrue(shotStarts.isEmpty());
        runUntil(() -> feeding, 40);
        assertTrue(followed.isEmpty());
        assertXY(57.8, 9, follower.poseAt(0));
        finishShot(1, 0);

        passThrough(auto.chegadinhafrente1(), 1, 1);
        passThrough(auto.prepcoletajardim(), 2, 1);
        passThrough(auto.coletajardim(), 3, 1);
        pose = auto.lancamento2().endPose();
        velocity = new Velocity(12, 0, 0);
        tick();
        tick();
        assertTrue(follower.isBusy());
        assertEquals(1, shotStarts.size());
        velocity = Velocity.zero();
        runUntil(() -> shotStarts.size() == 2, 40);
        assertEquals(1.0, Constants.foresightConfig.brakeAggression.get(), 1e-9);
        assertXY(60, 147, follower.poseAt(0));
        finishShot(2, 1);

        assertTrue("O coletor abre no inicio da aproximacao curva", flowerOpen);
        assertEquals(1.05, Constants.foresightConfig.brakeAggression.get(), 1e-9);
        pose = auto.prepcoletaflower().endPose();
        velocity = new Velocity(12, 0, 0);
        tick();
        tick();
        tickMillis(500);
        assertEquals("A preparacao espera parar mesmo com os servos prontos", 2, followed.size());
        velocity = Velocity.zero();
        runUntil(() -> followed.size() == 3, 40);
        assertEquals("A coleta usa a frenagem original", 1.0,
                Constants.foresightConfig.brakeAggression.get(), 1e-9);
        pose = auto.coletaflower().endPose();
        velocity = new Velocity(12, 0, 0);
        tick();
        tick();
        tickMillis(4000);
        assertEquals("A coleta nao inicia a espera enquanto o robo se move", 3, followed.size());
        velocity = Velocity.zero();
        runUntil(() -> follower.holding() && !follower.isBusy(), 10);
        long collectionStarted = clock.get();
        assertXY(46.6, 131, follower.poseAt(0));
        tickMillis(1999);
        assertEquals(3, followed.size());
        assertTrue(flowerOpen);
        assertFalse(feeding || transferring);
        tickMillis(1);
        assertEquals(4, followed.size());
        assertEquals("O retorno da flower usa a frenagem original", 1.0,
                Constants.foresightConfig.brakeAggression.get(), 1e-9);
        assertEquals(2000, millis(clock.get() - collectionStarted));

        pose = auto.posflower().endPose();
        velocity = new Velocity(12, 0, 0);
        tick();
        tick();
        assertTrue("O coletor so fecha depois que o retorno parar", flowerOpen);
        assertEquals(4, followed.size());
        velocity = Velocity.zero();
        runUntil(() -> !flowerOpen, 10);
        assertEquals(4, followed.size());
        tickMillis(499);
        assertEquals(4, followed.size());
        tickMillis(1);
        assertEquals(5, followed.size());
        assertEquals(1.05, Constants.foresightConfig.brakeAggression.get(), 1e-9);
        assertEquals(1, followed.get(4).getSegments().size());
        pose = auto.lancamento3().get(0.5);
        velocity = new Velocity(12, 0, 0);
        tick();
        assertTrue(follower.following());
        assertFalse(follower.holding());
        assertEquals(5, followed.size());
        pose = auto.lancamento3().endPose();
        tick();
        tick();
        assertTrue("O terceiro disparo tambem espera o robo parar", follower.isBusy());
        assertEquals(2, shotStarts.size());
        velocity = Velocity.zero();
        runUntil(() -> shotStarts.size() == 3, 40);
        assertXY(60, 147, follower.poseAt(0));
        finishShot(6, 2);
        assertEquals("O estacionamento usa a frenagem original", 1.0,
                Constants.foresightConfig.brakeAggression.get(), 1e-9);

        pose = auto.estacionamento().endPose();
        velocity = new Velocity(12, 0, 0);
        tick();
        tick();
        assertTrue(follower.isBusy());
        verify(intake, never()).offMotor();
        velocity = Velocity.zero();
        for (int i = 0; i < 5; i++) tick();
        assertTrue(follower.holding());
        assertXY(10, 120, follower.poseAt(0));
        assertEquals(Arrays.asList(0, 1, 5), shotGroupCounts);
        assertEquals(Arrays.asList(42, 38, 38), shotTags);
        assertEquals(3, shotStarts.size());
        assertEquals(3, shotEnds.size());
        assertFalse(flowerOpen || feeding || transferring);
        Path[] expected = paths();
        int[] groupSizes = {4, 1, 1, 1, 1, 1};
        int pathIndex = 0;
        for (int i = 0; i < followed.size(); i++) {
            List<PathSegment> segments = followed.get(i).getSegments();
            assertEquals(groupSizes[i], segments.size());
            for (PathSegment segment : segments) {
                for (double t : new double[] {0, 0.25, 0.5, 0.75, 1}) {
                    assertPose(expected[pathIndex].get(t),
                            segment.curve.get(t).toPose(segment.heading(t)));
                }
                pathIndex++;
            }
        }
        assertEquals(expected.length, pathIndex);
        verify(flowerCollect).setOpen();
        verify(flywheel, atLeastOnce()).runByDistance(OptionalDouble.of(1.77));
        verify(flywheel, atLeastOnce()).stopMotors();
        verify(intake, atLeastOnce()).offMotor();
    }

    @Test
    public void directFlowerApproachOpensCollectorOnceAndWaitsForServoMovement() {
        auto.startAutonomous();
        for (int i = 0; i < 800 && followed.size() < 2; i++) {
            if (follower.following() && follower.currentSegment() != null) {
                pose = follower.currentCurve().get(1).toPose(follower.currentSegment().heading(1));
            }
            tick();
        }
        assertEquals(2, followed.size());
        assertTrue("O coletor deve abrir ao iniciar a curva direta", flowerOpen);
        verify(flowerCollect).setOpen();
        assertEquals(1, followed.get(1).getSegments().size());
        assertPose(auto.lancamento2().endPose(), followed.get(1).get(0));

        pose = auto.prepcoletaflower().endPose();
        runUntil(() -> follower.holding() && !follower.isBusy(), 10);
        long elapsed = millis(clock.get() - openedAt);
        assertTrue(elapsed < 500);
        tickMillis(499 - elapsed);
        assertEquals("Mesmo chegando cedo, deve esperar os servos", 2, followed.size());
        tickMillis(1);
        assertEquals(3, followed.size());
        assertTrue(flowerOpen);
        verify(flowerCollect).setOpen();
    }

    @Test
    public void stoppingDuringTransitRestoresTheSharedBrakingConfiguration() {
        double originalAggression = Constants.foresightConfig.brakeAggression.get();
        auto.startAutonomous();
        runUntil(() -> feeding, 120);
        finishShot(1, 0);
        assertEquals(1.05, Constants.foresightConfig.brakeAggression.get(), 1e-9);

        auto.finishAutonomous(true);

        assertEquals(originalAggression, Constants.foresightConfig.brakeAggression.get(), 1e-9);
    }

    @Test
    public void updatesRpmByDistanceWhileLimelightAdjustsAim() {
        tx = 6;
        Pose initial = pose;
        auto.startAutonomous();
        tickMillis(SPIN_UP_MS);
        when(limelight.getDistanceMeters()).thenReturn(OptionalDouble.of(0.5));
        tick();
        when(limelight.getDistanceMeters()).thenReturn(OptionalDouble.of(3));
        tick();
        when(limelight.getDistanceMeters()).thenReturn(OptionalDouble.empty());
        tick();

        assertTrue(follower.holding());
        assertXY(initial.x(), initial.y(), follower.poseAt(0));
        assertAngle(Math.toDegrees(initial.heading()) - tx, follower.poseAt(0).heading());
        verify(limelight, atLeastOnce()).getObservation();
        InOrder distanceUpdates = inOrder(flywheel);
        distanceUpdates.verify(flywheel).runByDistance(OptionalDouble.of(0.5));
        distanceUpdates.verify(flywheel).runByDistance(OptionalDouble.of(3));
        distanceUpdates.verify(flywheel).runByDistance(OptionalDouble.empty());
        verify(flywheel, never()).run(anyDouble());
    }

    @Test
    public void aimsForHalfASecondThenShootsWithoutAlignmentOrRpmConfirmation() {
        tx = 6;
        firstRpm = 0;
        secondRpm = 0;
        auto.startAutonomous();
        tickMillis(SPIN_UP_MS);
        tickMillis(499);
        assertTrue(shotStarts.isEmpty());
        tickMillis(1);
        assertTrue(feeding && transferring);
        assertEquals(Long.valueOf(TimeUnit.MILLISECONDS.toNanos(SPIN_UP_MS + AIM_TIME_MS)), shotStarts.get(0));
        assertTrue(followed.isEmpty());
        verify(flywheel, never()).getRPM();
        verify(flywheel, never()).getSecondMotorRPM();
    }

    @Test
    public void visionLossDoesNotInterruptFeedingOrChangeTheHeldHeading() {
        auto.startAutonomous();
        runUntil(() -> feeding, 120);
        Pose held = follower.poseAt(0);
        visible = false;
        tick();
        assertTrue(feeding && transferring);
        assertTrue(follower.holding());
        assertTrue(followed.isEmpty());
        visible = true;
        tx = 10;
        tick();
        tick();
        assertTrue(feeding && transferring);
        assertPose(held, follower.poseAt(0));
        finishShot(1, 0);
    }

    @Test
    public void rpmLossDoesNotInterruptTheShotWindow() {
        auto.startAutonomous();
        runUntil(() -> feeding, 120);
        secondRpm = Double.NaN;
        tick();
        assertTrue(feeding && transferring);
        assertTrue(follower.holding());
        secondRpm = 2670;
        for (int i = 0; i < 7; i++) tick();
        assertTrue(feeding && transferring);
        finishShot(1, 0);
    }

    @Test
    public void redAllianceUsesOppositeCellInitiallyAndAudienceForBothReturnShots() throws Exception {
        inject("redAlliance", true);
        completeRoute();
        assertEquals(Arrays.asList(30, 34, 34), shotTags);
        assertEquals(Arrays.asList(0, 1, 5), shotGroupCounts);
        verify(limelight, atLeastOnce()).run(false, true, false, true);
        verify(limelight, atLeastOnce()).run(true, false, false, true);
    }

    @Test
    public void missingVisionStillFiresAllThreeShotsAndCompletesFlowerCollectionAndParking() {
        visible = false;
        completeRoute();
        assertEquals(Arrays.asList(0, 1, 5), shotGroupCounts);
        assertEquals(3, shotStarts.size());
        assertEquals(3, shotEnds.size());
        for (int i = 0; i < 3; i++) {
            assertShotDuration(i);
        }
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
                auto.lancamento2(), auto.prepcoletaflower(), auto.coletaflower(),
                auto.posflower(), auto.lancamento3(), auto.estacionamento()};
    }

    private void finishShot(int nextGroupCount, int shotIndex) {
        runUntil(() -> followed.size() == nextGroupCount, 200);
        assertShotDuration(shotIndex);
    }

    private void passThrough(Path path, int nextSegmentIndex, int groupCount) {
        pose = path.endPose();
        velocity = new Velocity(12, 0, 0);
        tick();
        assertTrue("A transicao deve continuar seguindo sem entrar em hold", follower.following());
        assertFalse(follower.holding());
        assertTrue("O robo ainda esta em movimento", follower.isBusy());
        assertEquals(nextSegmentIndex, follower.pathIndex());
        assertEquals("A transicao usa o mesmo comando follow", groupCount, followed.size());
    }

    private void assertShotDuration(int shotIndex) {
        long duration = millis(shotEnds.get(shotIndex) - shotStarts.get(shotIndex));
        assertTrue(duration >= SHOOT_TIME_MS && duration < SHOOT_TIME_MS + 20);
    }

    private void completeRoute() {
        auto.startAutonomous();
        for (int i = 0; i < 1600; i++) {
            if (follower.following() && follower.currentSegment() != null) {
                pose = follower.currentCurve().get(1).toPose(follower.currentSegment().heading(1));
            }
            tick();
            if (followed.size() == 6 && follower.holding() && !follower.isBusy()) {
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
        Field field = AutoPrincipal.class.getDeclaredField(name);
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
