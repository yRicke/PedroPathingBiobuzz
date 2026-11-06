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

public class AutoTerciarioTest {
    private final AtomicLong clock = new AtomicLong();
    private final Localizer localizer = mock(Localizer.class, CALLS_REAL_METHODS);
    private final Drivetrain drivetrain = mock(Drivetrain.class);
    private final Intake intake = mock(Intake.class);
    private final Flywheel flywheel = mock(Flywheel.class);
    private final Transfer transfer = mock(Transfer.class);
    private final Trigger trigger = mock(Trigger.class);
    private final Limelight limelight = mock(Limelight.class);
    private final FlowerCollect flowerCollect = mock(FlowerCollect.class);
    private final AutoTerciario auto = new AutoTerciario();
    private final List<Path> followed = new ArrayList<>();
    private final List<Long> shotStarts = new ArrayList<>();
    private final List<Long> shotEnds = new ArrayList<>();
    private final List<Integer> shotGroupCounts = new ArrayList<>();
    private final List<Integer> shotTags = new ArrayList<>();
    private Pose pose = new Pose(59.6258, 136.8065, Math.toRadians(270));
    private Velocity velocity = Velocity.zero();
    private Follower follower;
    private boolean feeding, transferring, flowerOpen, visible = true;
    private int selectedTag = 38;
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
            if (nextIndex == 1) {
                assertTrue("O coletor deve estar aberto antes de coletar", flowerOpen);
                assertTrue(millis(clock.get() - openedAt) >= 500);
            }
            if (nextIndex == 3) {
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
    public void preservesSuppliedFiveSegmentsWithDirectCurvedReturn() {
        Path[] paths = paths();
        assertEquals(5, paths.length);
        assertXY(59.6258, 136.8065, paths[0].get(0));
        double[][] ends = {{46.4006, 120.9045}, {46.6, 131}, {46.6, 120},
                {59.8174, 136.7026}, {10, 120}};
        int[] sizes = {1, 1, 1, 1, 1};
        for (int i = 0; i < paths.length; i++) {
            assertEquals(sizes[i], paths[i].getSegments().size());
            assertXY(ends[i][0], ends[i][1], paths[i].endPose());
            if (i > 0) assertXY(ends[i - 1][0], ends[i - 1][1], paths[i].get(0));
        }
        // Amostras calculadas a partir dos controles fornecidos no exportado.
        assertXY(60.42095, 122.76225, paths[0].get(0.5));
        assertXY(50.31885, 121.2321, paths[4].get(0.5));
        assertAngle(270, paths[0].get(0).heading());
        assertAngle(90, paths[0].endPose().heading());
        assertAngle(90, paths[1].get(0.5).heading());
        assertAngle(90, paths[2].get(0.5).heading());
        List<PathSegment> returning = paths[3].getSegments();
        assertEquals(1, returning.size());
        assertXY(46.6, 120, returning.get(0).curve.get(0).toPose(0));
        assertXY(59.8174, 136.7026, returning.get(0).curve.get(1).toPose(0));
        assertXY(60.61, 122.87535, paths[3].get(0.5));
        double reverseHeading = Math.atan2(136.7026 - 120, 59.8174 - 46.6) + Math.PI;
        assertAngle(Math.toDegrees(reverseHeading), returning.get(0).heading(0.5));
        assertAngle(-66.9996, paths[3].endPose().heading());
        // O exportado reinicia o heading do estacionamento em 270 graus.
        assertAngle(270, paths[4].get(0).heading());
        assertAngle(0, paths[4].endPose().heading());
    }

    @Test
    public void firesTwiceAndCollectsBeforeReturningAndParking() {
        startAfterInitialDelay();
        assertFalse(flowerOpen);
        tickMillis(1499);
        assertTrue(followed.isEmpty());
        assertTrue(shotStarts.isEmpty());
        runUntil(() -> feeding, 40);
        assertXY(59.6258, 136.8065, follower.poseAt(0));
        assertEquals(38, selectedTag);
        finishShot(1, 0);

        assertTrue("O coletor abre ao sair da posicao inicial", flowerOpen);
        assertEquals(1, followed.get(0).getSegments().size());
        assertEquals(1.05, Constants.foresightConfig.brakeAggression.get(), 1e-9);
        pose = auto.preparacaoflower().endPose();
        velocity = new Velocity(12, 0, 0);
        tick();
        tick();
        tickMillis(500);
        assertEquals("A aproximacao aguarda a parada", 1, followed.size());
        velocity = Velocity.zero();
        runUntil(() -> followed.size() == 2, 40);
        assertEquals(1.0, Constants.foresightConfig.brakeAggression.get(), 1e-9);

        pose = auto.coletaflower().endPose();
        velocity = new Velocity(12, 0, 0);
        tick();
        tick();
        tickMillis(4000);
        assertEquals("A espera de coleta so comeca parado", 2, followed.size());
        velocity = Velocity.zero();
        runUntil(() -> follower.holding() && !follower.isBusy(), 10);
        assertXY(46.6, 131, follower.poseAt(0));
        tickMillis(1999);
        assertEquals(2, followed.size());
        assertTrue(flowerOpen);
        assertFalse(feeding || transferring);
        tickMillis(1);
        assertEquals(3, followed.size());

        pose = auto.posflower().endPose();
        velocity = new Velocity(12, 0, 0);
        tick();
        tick();
        assertTrue(flowerOpen);
        velocity = Velocity.zero();
        runUntil(() -> !flowerOpen, 10);
        tickMillis(499);
        assertEquals(3, followed.size());
        tickMillis(1);
        assertEquals(4, followed.size());
        assertEquals(1.05, Constants.foresightConfig.brakeAggression.get(), 1e-9);

        Path returning = auto.retornolancamento3();
        assertEquals(1, returning.getSegments().size());
        pose = returning.get(0.5);
        velocity = new Velocity(12, 0, 0);
        tick();
        assertTrue(follower.following());
        assertFalse(follower.holding());
        assertEquals(0, follower.pathIndex());
        assertEquals(4, followed.size());
        pose = returning.endPose();
        tick();
        tick();
        assertTrue(follower.isBusy());
        assertEquals(1, shotStarts.size());
        velocity = Velocity.zero();
        runUntil(() -> shotStarts.size() == 2, 40);
        assertXY(59.8174, 136.7026, follower.poseAt(0));
        finishShot(5, 1);
        assertEquals(1.0, Constants.foresightConfig.brakeAggression.get(), 1e-9);

        pose = auto.estacionamento().endPose();
        velocity = new Velocity(12, 0, 0);
        tick();
        tick();
        verify(intake, times(1)).offMotor();
        velocity = Velocity.zero();
        for (int i = 0; i < 5; i++) tick();
        assertTrue(follower.holding());
        assertXY(10, 120, follower.poseAt(0));
        assertEquals(Arrays.asList(0, 4), shotGroupCounts);
        assertEquals(Arrays.asList(38, 38), shotTags);
        assertEquals(2, shotStarts.size());
        assertEquals(2, shotEnds.size());
        assertFalse(flowerOpen || feeding || transferring);
        Path[] expected = paths();
        assertEquals(expected.length, followed.size());
        for (int i = 0; i < expected.length; i++) {
            for (double t : new double[] {0, 0.25, 0.5, 0.75, 1}) {
                assertPose(expected[i].get(t), followed.get(i).get(t));
            }
        }
        verify(flowerCollect).setOpen();
        verify(flywheel, atLeastOnce()).run(AutonomousConstants.FLYWHEEL_RPM);
        verify(flywheel, never()).runByDistance(any());
        verify(flywheel, atLeastOnce()).stopMotors();
        verify(intake, atLeastOnce()).offMotor();
    }

    @Test
    public void earlyArrivalStillWaitsHalfASecondForTheOpenServos() {
        startAfterInitialDelay();
        runUntil(() -> feeding, 120);
        finishShot(1, 0);
        assertTrue(flowerOpen);
        verify(flowerCollect).setOpen();
        pose = auto.preparacaoflower().endPose();
        runUntil(() -> follower.holding() && !follower.isBusy(), 10);
        long elapsed = millis(clock.get() - openedAt);
        assertTrue(elapsed < 500);
        tickMillis(499 - elapsed);
        assertEquals(1, followed.size());
        tickMillis(1);
        assertEquals(2, followed.size());
        assertTrue(flowerOpen);
        verify(flowerCollect).setOpen();
    }

    @Test
    public void stoppingDuringTransitRestoresTheSharedBrakingConfiguration() {
        double originalAggression = Constants.foresightConfig.brakeAggression.get();
        startAfterInitialDelay();
        runUntil(() -> feeding, 120);
        finishShot(1, 0);
        assertEquals(1.05, Constants.foresightConfig.brakeAggression.get(), 1e-9);

        auto.finishAutonomous(true);

        assertEquals(originalAggression, Constants.foresightConfig.brakeAggression.get(), 1e-9);
    }

    @Test
    public void fixedRpmIgnoresDistanceWhileLimelightStillAdjustsAim() {
        tx = 6;
        Pose initial = pose;
        startAfterInitialDelay();
        tickMillis(1500);
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
        verify(limelight, never()).getDistanceMeters();
        verify(flywheel, atLeastOnce()).run(2560);
        verify(flywheel, never()).run(doubleThat(rpm -> rpm != 2560));
        verify(flywheel, never()).runByDistance(any());
    }

    @Test
    public void aimsForHalfASecondThenShootsWithoutAlignmentOrRpmConfirmation() {
        tx = 6;
        firstRpm = 0;
        secondRpm = 0;
        startAfterInitialDelay();
        tickMillis(1500);
        tickMillis(499);
        assertTrue(shotStarts.isEmpty());
        tickMillis(1);
        assertTrue(feeding && transferring);
        assertEquals(Long.valueOf(TimeUnit.MILLISECONDS.toNanos(10000)), shotStarts.get(0));
        assertTrue(followed.isEmpty());
        verify(flywheel, never()).getRPM();
        verify(flywheel, never()).getSecondMotorRPM();
    }

    @Test
    public void visionLossDoesNotInterruptFeedingOrChangeTheHeldHeading() {
        startAfterInitialDelay();
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
    public void rpmLossDoesNotInterruptTheTwoSecondShotWindow() {
        startAfterInitialDelay();
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
    public void redAllianceUsesAudienceCellForBothShots() throws Exception {
        inject("redAlliance", true);
        completeRoute();
        assertEquals(Arrays.asList(34, 34), shotTags);
        assertEquals(Arrays.asList(0, 4), shotGroupCounts);
        verify(limelight, atLeastOnce()).run(true, false, false, true);
    }

    @Test
    public void missingVisionStillFiresTwiceAndCompletesFlowerCollectionAndParking() {
        visible = false;
        completeRoute();
        assertEquals(Arrays.asList(0, 4), shotGroupCounts);
        assertEquals(2, shotStarts.size());
        assertEquals(2, shotEnds.size());
        for (int i = 0; i < 2; i++) {
            assertEquals(2000, millis(shotEnds.get(i) - shotStarts.get(i)));
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

    @Test
    public void waitsEightSecondsBeforeWarmingAimingOrMoving() {
        auto.startAutonomous();
        tickMillis(7999);
        assertTrue(follower.idle());
        assertTrue(followed.isEmpty());
        assertTrue(shotStarts.isEmpty());
        assertFalse(flowerOpen || feeding || transferring);
        verify(flywheel, never()).run(anyDouble());
        verify(intake, never()).onMotor();
        verify(limelight, never()).getObservation();

        tickMillis(1);
        assertTrue(follower.holding());
        verify(flywheel).run(2560);
        verify(intake).onMotor();
        tickMillis(1499);
        assertTrue(shotStarts.isEmpty());
        tickMillis(1);
        tickMillis(499);
        assertTrue(shotStarts.isEmpty());
        tickMillis(1);
        assertEquals(Long.valueOf(TimeUnit.MILLISECONDS.toNanos(10000)), shotStarts.get(0));
        assertTrue(followed.isEmpty());
        tickMillis(1999);
        assertTrue(followed.isEmpty());
        tickMillis(1);
        assertEquals(1, followed.size());
    }

    @Test
    public void stopDuringInitialDelayDoesNotStartTheRoutine() {
        auto.startAutonomous();
        tickMillis(4000);
        auto.finishAutonomous(true);
        assertTrue(follower.idle());
        assertTrue(followed.isEmpty());
        assertTrue(shotStarts.isEmpty());
        assertFalse(flowerOpen || feeding || transferring);
        verify(flywheel, never()).run(anyDouble());
        verify(intake, never()).onMotor();
        verify(limelight).stop();
        verify(flywheel, atLeastOnce()).stopMotors();
    }

    private void startAfterInitialDelay() {
        auto.startAutonomous();
        tickMillis(8000);
    }

    private Path[] paths() {
        return new Path[] {auto.preparacaoflower(), auto.coletaflower(), auto.posflower(),
                auto.retornolancamento3(), auto.estacionamento()};
    }

    private void finishShot(int nextMovementCount, int shotIndex) {
        runUntil(() -> followed.size() == nextMovementCount, 200);
        assertEquals(2000, millis(shotEnds.get(shotIndex) - shotStarts.get(shotIndex)));
    }

    private void completeRoute() {
        startAfterInitialDelay();
        for (int i = 0; i < 1600; i++) {
            if (follower.following() && follower.currentSegment() != null) {
                pose = follower.currentCurve().get(1).toPose(follower.currentSegment().heading(1));
            }
            tick();
            if (followed.size() == 5 && follower.holding() && !follower.isBusy()) {
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
        Field field = AutoTerciario.class.getDeclaredField(name);
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
