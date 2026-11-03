package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.junit.Before;
import org.junit.Test;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.AprilTagObservation;

import java.util.Collections;
import java.util.Map;
import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

public class PedroDriveTest {
    private static final double EPSILON = 1e-9;

    private final SimulatedLocalizer localizer = new SimulatedLocalizer();
    private final RecordingDrivetrain drivetrain = new RecordingDrivetrain();
    private final AtomicLong clock = new AtomicLong();
    private Follower follower;
    private PedroDrive drive;

    @Before
    public void setUp() {
        follower = new Follower(localizer, drivetrain, new Foresight(Constants.foresightConfig));
        drive = new PedroDrive(follower, null, Controller.proportional(1), clock::get);
    }

    @Test
    public void forwardRemainsFieldRelativeAfterQuarterTurn() {
        localizer.setPose(new Pose(0, 0, Math.PI / 2));

        drive.run(1, 0, 0, false);

        assertPowers(0, -1, 0);
        assertTrue(drivetrain.manual);
    }

    @Test
    public void rightStrafeAndClockwiseTurnKeepDriverDirections() {
        drive.run(0, 1, 0.5, false);

        assertPowers(0, -1, -0.5);
    }

    @Test
    public void precisionScalesTranslationAndTurn() {
        drive.run(1, 0.5, 0.5, true);

        assertPowers(0.3, -0.15, -0.15);
    }

    @Test
    public void driverCanLeaveHoldWithSmallTurnInPrecisionMode() {
        drive.run(0, 0, 0, true);
        assertTrue(follower.holding());

        drive.run(0, 0, 0.2, true);

        assertTrue(follower.manual());
        assertPowers(0, 0, -0.06);
    }

    @Test
    public void releasedControlsHoldStoppedPoseUntilDrivingResumes() {
        Pose stoppedPose = new Pose(10, 20, 0.6);
        localizer.setPose(stoppedPose);
        localizer.velocity = new Velocity(1, 0, 0);
        drive.run(0.5, 0, 0, false);

        drive.run(0, 0, 0, false);
        assertTrue(follower.manual());
        assertPowers(0, 0, 0);

        localizer.velocity = Velocity.zero();
        drive.run(0, 0, 0, false);
        assertTrue(follower.holding());
        assertSame(stoppedPose, follower.poseAt(0));

        localizer.setPose(new Pose(12, 19, 0.8));
        drive.run(0, 0, 0, false);
        assertTrue(follower.holding());
        assertSame(stoppedPose, follower.poseAt(0));

        drive.run(0.5, 0, 0, false);
        assertTrue(follower.manual());
        assertTrue(drivetrain.manual);
    }

    @Test
    public void stopImmediatelyRemovesMotorPower() {
        drive.run(1, 0, 0, false);

        drive.stop();

        assertTrue(follower.idle());
        assertPowers(0, 0, 0);
    }

    @Test
    public void aimDoesNotHoldXYWhileCorrectingHeading() {
        localizer.setPose(new Pose(10, 20, Math.toRadians(90)));

        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 10, 1, 1));

        assertTrue(follower.manual());
        assertPowers(0, 0, Math.toRadians(-8));

        localizer.setPose(new Pose(11, 19, Math.toRadians(86)));
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 3, 1, 2));
        assertPowers(0, 0, Math.toRadians(-3));
    }

    @Test
    public void aimPreservesFieldCentricTranslationAndOverridesDriverYaw() {
        localizer.setPose(new Pose(10, 20, Math.PI / 2));

        drive.aim(0.6, 0.4, -0.8, false,
                AprilTagObservation.detected(42, 6, 1, 1));

        assertTrue(follower.manual());
        assertPowers(-0.4, -0.6, Math.toRadians(-6));
        assertTrue(drivetrain.manual);
    }

    @Test
    public void aimPrecisionScalesTranslationAndCorrection() {
        drive.aim(0.6, 0.4, 1, true,
                AprilTagObservation.detected(42, -6, 1, 1));

        assertPowers(0.18, -0.12, Math.toRadians(1.8));
    }

    @Test
    public void missingTargetDoesNotStartAutomaticSearch() {
        drive.aim(0, 0, 0, false, AprilTagObservation.searching(42, 1));

        assertEquals(0, drivetrain.powers.turn(), EPSILON);

        localizer.setPose(new Pose(5.5, 6.5, Math.toRadians(-1)));
        drive.aim(0, 0, 0.5, false, AprilTagObservation.searching(42, 2));
        assertPowers(0, 0, -0.5);
    }

    @Test
    public void repeatedImageDoesNotAccumulateHeadingCorrections() {
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 6, 1, 1));
        assertPowers(0, 0, Math.toRadians(-6));

        localizer.setPose(new Pose(0, 0, Math.toRadians(-2)));
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 6, 1, 1));
        assertPowers(0, 0, Math.toRadians(-4));

        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 3, 1, 2));
        assertPowers(0, 0, Math.toRadians(-3));
    }

    @Test
    public void centeredHiveKeepsTrackingWithoutHoldingPosition() {
        localizer.setPose(new Pose(1, 2, Math.toRadians(5)));
        drive.aim(0, 0, 0.8, false, AprilTagObservation.detected(42, 0.5, 1, 1));
        assertTrue(follower.manual());
        assertPowers(0, 0, 0);

        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, -3, 1, 2));
        assertPowers(0, 0, Math.toRadians(3));
    }

    @Test
    public void lostTargetImmediatelyReturnsYawToDriverWhileBumperStaysPressed() {
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, -4, 1, 1));

        drive.aim(0.6, 0.4, 0.5, false, AprilTagObservation.searching(42, 2));

        assertTrue(follower.manual());
        assertPowers(0.6, -0.4, -0.5);
    }

    @Test
    public void disconnectedVisionReturnsAllAxesToDriverInPrecisionMode() {
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, -4, 1, 1));

        drive.aim(0.6, 0.4, 0.5, true, AprilTagObservation.unavailable(42));

        assertPowers(0.18, -0.12, -0.15);
    }

    @Test
    public void lostTargetWithNeutralSticksCancelsPreviousCorrection() {
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 8, 1, 1));
        localizer.setPose(new Pose(5, 6, 0.35));

        drive.aim(0, 0, 0, false, AprilTagObservation.unavailable(42));

        assertHoldAt(5, 6, 0.35);
        assertPowers(0, 0, 0);
    }

    @Test
    public void reacquiredTargetUsesCurrentHeadingEvenIfTimestampRepeats() {
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 6, 1, 1));
        drive.aim(0, 0, 0.5, false, AprilTagObservation.unavailable(42));
        localizer.setPose(new Pose(25, 30, Math.toRadians(80)));

        drive.aim(0.4, 0, 0, false, AprilTagObservation.detected(42, 3, 1, 1));

        assertTrue(follower.manual());
        assertEquals(Math.toRadians(-3), drivetrain.powers.turn(), EPSILON);
    }

    @Test
    public void releaseWithNeutralSticksCancelsPreviousTurn() {
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 12, 1, 1));

        drive.run(0, 0, 0, false);

        assertHoldAt(0, 0, 0);
        assertPowers(0, 0, 0);
    }

    @Test
    public void releaseAllowsManualDriveAndNextAimUsesNewHeading() {
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 6, 1, 1));

        drive.run(0.6, 0, -0.5, false);
        assertTrue(follower.manual());
        assertPowers(0.6, 0, 0.5);

        localizer.setPose(new Pose(25, 30, Math.toRadians(90)));
        drive.aim(0.6, 0, 0, false, AprilTagObservation.detected(42, -3, 1, 1));
        assertPowers(0, -0.6, Math.toRadians(3));
    }

    @Test
    public void changingHiveProcessesItsOffsetEvenWithinSameImage() {
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 6, 1, 1));
        localizer.setPose(new Pose(0, 0, Math.toRadians(-1)));

        drive.aim(0, 0, 0, false, AprilTagObservation.detected(38, -3, 1, 1));

        assertPowers(0, 0, Math.toRadians(3));
    }

    @Test
    public void allianceChangeProcessesNewCellEvenWithinSameImage() {
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 6, 1, 1));
        localizer.setPose(new Pose(0, 0, Math.toRadians(-1)));
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(30, -3, 1, 1));
        assertPowers(0, 0, Math.toRadians(3));
    }

    @Test
    public void aimTakesOverFromAnExistingPoseHold() {
        drive.run(0, 0, 0, false);
        assertTrue(follower.holding());

        drive.aim(0.6, 0, 0, false, AprilTagObservation.detected(42, 0, 1, 1));

        assertTrue(follower.manual());
        assertPowers(0.6, 0, 0);
    }

    @Test
    public void aimNormalizesHeadingAcrossWraparound() {
        localizer.setPose(new Pose(0, 0, Math.toRadians(-179)));

        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 4, 1, 1));

        assertPowers(0, 0, Math.toRadians(-4));
    }

    @Test
    public void stopClearsAimHeadingForNextActivation() {
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 4, 1, 1));
        drive.stop();
        localizer.setPose(new Pose(7, 8, Math.toRadians(90)));

        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, -3, 1, 1));

        assertPowers(0, 0, Math.toRadians(3));
    }

    @Test
    public void fieldResetChangesControlsWithoutChangingGlobalPose() {
        Pose globalPose = new Pose(8.4, 35.2, Math.PI / 2);
        localizer.setPose(globalPose);

        drive.resetFieldCentric();
        drive.run(1, 0, 0, false);

        assertSame(globalPose, follower.pose());
        assertPowers(1, 0, 0);

        localizer.setPose(new Pose(8.4, 35.2, Math.PI));
        drive.run(1, 0, 0, false);
        assertPowers(0, -1, 0);
    }

    @Test
    public void fieldResetAlsoAppliesToTranslationDuringVisionAim() {
        localizer.setPose(new Pose(8.4, 35.2, Math.PI / 2));
        drive.resetFieldCentric();

        drive.aim(0.6, 0.4, 0.8, false,
                AprilTagObservation.detected(42, 6, 1, 1));

        assertPowers(0.6, -0.4, Math.toRadians(-6));
        assertEquals(Math.PI / 2, follower.pose().heading(), EPSILON);
    }

    @Test
    public void relocalizationCancelsOldHoldAndRestoresGlobalFieldReference() {
        localizer.setPose(new Pose(50, 60, Math.PI / 2));
        drive.resetFieldCentric();
        drive.run(0, 0, 0, false);
        assertTrue(follower.holding());

        drive.resetPose(new Pose(8.4, 35.2, Math.PI / 2));

        assertTrue(follower.idle());
        assertPowers(0, 0, 0);
        drive.run(0, 0, 0, false);
        assertHoldAt(8.4, 35.2, Math.PI / 2);

        drive.run(1, 0, 0, false);
        assertPowers(0, -1, 0);
    }

    @Test
    public void relocalizationClearsPreviousAimEvenWhenFrameTimestampRepeats() {
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 6, 1, 1));

        drive.resetPose(new Pose(8.4, 35.2, Math.PI / 2));
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 3, 1, 1));

        assertPowers(0, 0, Math.toRadians(-3));
    }

    @Test
    public void coordinateAimPreservesTranslationOverridesYawAndDoesNotHoldXY() {
        localizer.setPose(new Pose(57.8, 35.2, Math.PI / 2 - Math.toRadians(3)));

        drive.aimAt(0.6, 0.4, -0.8, false, 57.8, 50);

        double heading = follower.pose().heading();
        assertTrue(follower.manual());
        assertPowers(0.6 * Math.cos(heading) - 0.4 * Math.sin(heading),
                -0.6 * Math.sin(heading) - 0.4 * Math.cos(heading), Math.toRadians(3));
    }

    @Test
    public void coordinateAimUpdatesBearingAsRobotMovesAndHiveChanges() {
        localizer.setPose(new Pose(50, 50, 0));
        drive.aimAt(0, 0, 0.5, false, 57.8, 50);
        assertPowers(0, 0, 0);
        assertTrue(follower.manual());

        localizer.setPose(new Pose(50, 51, 0));
        drive.aimAt(0, 0, 0.5, false, 57.8, 50);
        assertPowers(0, 0, Math.atan2(-1, 7.8));

        drive.aimAt(0, 0, -0.5, false, 57.8, 90);
        assertPowers(0, 0, Math.toRadians(8));
    }

    @Test
    public void coordinateAimUsesShortestTurnAcrossAngleWraparound() {
        localizer.setPose(new Pose(0, 0, Math.toRadians(179)));

        drive.aimAt(0, 0, 0, false, -10, -10 * Math.tan(Math.toRadians(2)));

        assertPowers(0, 0, Math.toRadians(3));
    }

    @Test
    public void fieldResetDoesNotRotateGlobalHiveCoordinates() {
        localizer.setPose(new Pose(50, 70, Math.PI / 2));
        drive.resetFieldCentric();

        drive.aimAt(0.6, 0.4, 0, false, 57.8, 50);

        assertPowers(0.6, -0.4, Math.toRadians(-8));
        assertEquals(Math.PI / 2, follower.pose().heading(), EPSILON);
    }

    @Test
    public void coordinateAimPrecisionScalesTranslationAndCorrection() {
        drive.aimAt(0.6, 0.4, 1, true, 10, 10 * Math.tan(Math.toRadians(-6)));

        assertPowers(0.18, -0.12, Math.toRadians(-1.8));
    }

    @Test
    public void coordinateAimInsideToleranceIgnoresManualYawUntilReleased() {
        drive.aimAt(0, 0, 0.8, false, 10, 10 * Math.tan(Math.toRadians(0.5)));

        assertTrue(follower.manual());
        assertPowers(0, 0, 0);

        drive.run(0, 0, 0.5, false);
        assertPowers(0, 0, -0.5);
        drive.run(0, 0, 0, false);
        assertHoldAt(0, 0, 0);
    }

    @Test
    public void coincidentOrInvalidTargetReturnsYawToDriver() {
        drive.aimAt(0, 0, 0, false, 10, 10);
        localizer.setPose(new Pose(10, 10, 0));

        drive.aimAt(0.6, 0.4, 0.5, false, 10, 10);
        assertPowers(0.6, -0.4, -0.5);

        drive.aimAt(0.6, 0.4, 0.5, false, Double.NaN, 10);
        assertPowers(0.6, -0.4, -0.5);

        drive.aimAt(0, 0, 0, false, 10, 10);
        assertHoldAt(10, 10, 0);
        assertPowers(0, 0, 0);
    }

    @Test
    public void switchingAimSourcesReacquiresVisionEvenForRepeatedFrame() {
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 6, 1, 1));
        drive.aimAt(0, 0, 0, false, 10, 10);
        assertPowers(0, 0, Math.toRadians(8));
        localizer.setPose(new Pose(1, 2, Math.PI / 2));

        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 3, 1, 1));

        assertPowers(0, 0, Math.toRadians(-3));
    }

    @Test
    public void autonomousHeadingOf23DegreesDoesNotBecomeNewFieldForward() {
        OpModeStorage.saveAutonomousPose(new Pose(11, 42, Math.toRadians(23)));
        drive.resetPose(OpModeStorage.consumeAutonomousPose());

        drive.run(1, 0, 0, false);

        assertEquals(Math.toRadians(23), follower.pose().heading(), EPSILON);
        assertEquals(Math.toRadians(23), drive.fieldCentricHeading(), EPSILON);
        assertPowers(Math.cos(Math.toRadians(23)), -Math.sin(Math.toRadians(23)), 0);

        drive.aimAt(0, 0, 0.5, false, 21, 42 + 10 * Math.tan(Math.toRadians(26)));
        assertPowers(0, 0, Math.toRadians(3));
    }

    @Test
    public void hybridUsesCoordinatesUntilThreeDistinctVisionFramesAreConfirmed() {
        hybrid(AprilTagObservation.unavailable(42));
        assertPowers(0, 0, Math.toRadians(8));

        for (int i = 0; i < 20; i++) {
            hybrid(AprilTagObservation.detected(42, 6, 1, 1));
            assertPowers(0, 0, Math.toRadians(8));
        }
        hybrid(AprilTagObservation.detected(42, 6, 1, 2));
        assertPowers(0, 0, Math.toRadians(8));
        hybrid(AprilTagObservation.detected(42, 6, 1, 3));
        assertPowers(0, 0, Math.toRadians(-6));
        assertTrue(follower.manual());
    }

    @Test
    public void hybridAcquisitionStartsOverWhenTargetDisappearsBetweenFrames() {
        hybrid(AprilTagObservation.detected(42, 6, 1, 1));
        hybrid(AprilTagObservation.detected(42, 6, 1, 2));
        hybrid(AprilTagObservation.searching(42, 3));

        hybrid(AprilTagObservation.detected(42, 6, 1, 4));
        hybrid(AprilTagObservation.detected(42, 6, 1, 5));
        assertPowers(0, 0, Math.toRadians(8));
        hybrid(AprilTagObservation.detected(42, 6, 1, 6));
        assertPowers(0, 0, Math.toRadians(-6));
    }

    @Test
    public void hybridHoldsLastGlobalHeadingBrieflyThenSearchesAndReconfirmsVision() {
        lockHybridVision();
        localizer.setPose(new Pose(0, 0, Math.toRadians(-2)));

        hybrid(AprilTagObservation.unavailable(42));
        assertPowers(0, 0, Math.toRadians(-4));
        clock.set(99_000_000);
        hybrid(AprilTagObservation.unavailable(42));
        assertPowers(0, 0, Math.toRadians(-4));

        clock.set(100_000_000);
        hybrid(AprilTagObservation.unavailable(42));
        assertPowers(0, 0, Math.toRadians(8));

        hybrid(AprilTagObservation.detected(42, 3, 1, 4));
        hybrid(AprilTagObservation.detected(42, 3, 1, 5));
        assertPowers(0, 0, Math.toRadians(8));
        hybrid(AprilTagObservation.detected(42, 3, 1, 6));
        assertPowers(0, 0, Math.toRadians(-3));
    }

    @Test
    public void hybridReturnsDirectlyToVisionWhenShortDropoutEnds() {
        lockHybridVision();
        hybrid(AprilTagObservation.searching(42, 4));
        clock.set(50_000_000);

        hybrid(AprilTagObservation.detected(42, 3, 1, 5));

        assertPowers(0, 0, Math.toRadians(-3));
    }

    @Test
    public void hybridTargetChangeImmediatelyDropsOldHeadingAndRejectsOtherCellFrames() {
        lockHybridVision();
        for (int i = 4; i < 10; i++) {
            drive.aimHybrid(0, 0, 0.8, false, 10, -10, 38,
                    AprilTagObservation.detected(42, 6, 1, i));
            assertPowers(0, 0, Math.toRadians(-8));
        }
        for (int i = 1; i <= 3; i++) {
            drive.aimHybrid(0, 0, 0.8, false, 10, -10, 38,
                    AprilTagObservation.detected(38, -3, 1, i));
            assertPowers(0, 0, Math.toRadians(i < 3 ? -8 : 3));
        }
    }

    @Test
    public void hybridAllianceChangeKeepsCoordinatesButReconfirmsTheNewTags() {
        lockHybridVision();
        for (int i = 3; i <= 5; i++) {
            drive.aimHybrid(0, 0, 0.8, false, 10, 10, 30,
                    AprilTagObservation.detected(30, -3, 1, i));
            assertPowers(0, 0, Math.toRadians(i < 5 ? 8 : 3));
        }
    }

    @Test
    public void hybridReleaseRestoresManualYawAndNextActivationReconfirmsVision() {
        lockHybridVision();

        drive.run(0.4, 0, -0.5, false);
        assertPowers(0.4, 0, 0.5);
        hybrid(AprilTagObservation.detected(42, 6, 1, 3));
        hybrid(AprilTagObservation.detected(42, 6, 1, 4));
        assertPowers(0, 0, Math.toRadians(8));
        hybrid(AprilTagObservation.detected(42, 6, 1, 5));
        assertPowers(0, 0, Math.toRadians(-6));

        drive.run(0, 0, 0, false);
        assertHoldAt(0, 0, 0);
        assertPowers(0, 0, 0);
    }

    @Test
    public void hybridRelocalizationClearsVisionAndPreservesTheNewGlobalHeading() {
        lockHybridVision();
        drive.resetPose(new Pose(8.4, 35.2, Math.PI / 2));

        for (int i = 3; i <= 5; i++) {
            drive.aimHybrid(0, 0, 0.8, false, 8.4, 50, 42,
                    AprilTagObservation.detected(42, 3, 1, i));
            assertPowers(0, 0, i < 5 ? 0 : Math.toRadians(-3));
        }
        assertEquals(Math.PI / 2, drive.fieldCentricHeading(), EPSILON);
    }

    @Test
    public void hybridPrecisionAndFieldResetApplyDuringBothAimStages() {
        localizer.setPose(new Pose(8.4, 35.2, Math.PI / 2));
        drive.resetFieldCentric();
        for (int i = 1; i <= 3; i++) {
            drive.aimHybrid(0.6, 0.4, 0.8, true, 8.4, 50, 42,
                    AprilTagObservation.detected(42, 6, 1, i));
            assertPowers(0.18, -0.12, i < 3 ? 0 : Math.toRadians(-1.8));
        }
        assertEquals(Math.PI / 2, follower.pose().heading(), EPSILON);
    }

    @Test
    public void hybridCameraTimestampRestartRequiresThreeNewFrames() {
        for (int i = 100; i <= 102; i++) hybrid(AprilTagObservation.detected(42, 6, 1, i));
        assertPowers(0, 0, Math.toRadians(-6));

        hybrid(AprilTagObservation.detected(42, 6, 1, 1));
        hybrid(AprilTagObservation.detected(42, 6, 1, 2));
        assertPowers(0, 0, Math.toRadians(8));
        hybrid(AprilTagObservation.detected(42, 6, 1, 3));
        assertPowers(0, 0, Math.toRadians(-6));
    }

    @Test
    public void hybridCanAcquireVisionEvenWhenCoordinatesCannotGiveADirection() {
        for (double targetX : new double[] {0, Double.NaN}) {
            drive.stop();
            for (int i = 1; i <= 3; i++) {
                drive.aimHybrid(0.4, 0.2, 0.5, false, targetX, 0, 42,
                        AprilTagObservation.detected(42, 3, 1, i));
                assertPowers(0.4, -0.2, i < 3 ? -0.5 : Math.toRadians(-3));
            }
        }
    }

    @Test
    public void hybridHoldKeepsEndpointXYWhileBothAimSourcesAdjustHeading() {
        Pose endpoint = new Pose(10, 20, 0);
        localizer.setPose(endpoint);
        for (int i = 1; i <= 3; i++) {
            drive.aimHybridHold(endpoint, 10, 30, 42,
                    AprilTagObservation.detected(42, 6, 1, i));
            assertHoldAt(10, 20, Math.toRadians(i < 3 ? 8 : -6));
        }

        localizer.setPose(new Pose(11, 19, Math.toRadians(1)));
        drive.aimHybridHold(endpoint, 10, 30, 42,
                AprilTagObservation.detected(42, 3, 1, 4));
        assertHoldAt(10, 20, Math.toRadians(-2));
    }

    @Test
    public void hybridAlignmentRequiresConfirmedVisionAndIsClearedDuringLossOrRelease() {
        Pose endpoint = Pose.zero();
        for (int i = 1; i <= 3; i++) {
            drive.aimHybridHold(endpoint, 10, 0, 42,
                    AprilTagObservation.detected(42, 0.5, 1, i));
            assertEquals(i == 3, drive.isHybridVisionAligned());
        }
        drive.aimHybridHold(endpoint, 10, 0, 42,
                AprilTagObservation.detected(42, 6, 1, 4));
        assertFalse(drive.isHybridVisionAligned());
        drive.aimHybridHold(endpoint, 10, 0, 42,
                AprilTagObservation.detected(42, 0, 1, 5));
        assertTrue(drive.isHybridVisionAligned());

        drive.aimHybridHold(endpoint, 10, 0, 42, AprilTagObservation.unavailable(42));
        assertFalse(drive.isHybridVisionAligned());
        assertTrue(follower.holding());

        drive.aimHybridHold(endpoint, 10, 0, 42,
                AprilTagObservation.detected(42, 0, 1, 6));
        assertTrue(drive.isHybridVisionAligned());
        drive.run(0, 0, 0, false);
        assertFalse(drive.isHybridVisionAligned());
    }

    @Test
    public void teleOpHybridRestoresFreeTranslationAfterAnAutonomousAimHold() {
        drive.aimHybridHold(Pose.zero(), 10, 10, 42, AprilTagObservation.unavailable(42));
        assertTrue(follower.holding());

        drive.aimHybrid(0.6, 0.4, 0.8, false, 10, 10, 42, AprilTagObservation.unavailable(42));

        assertTrue(follower.manual());
        assertPowers(0.6, -0.4, Math.toRadians(8));
    }

    @Test
    public void hybridHoldWithInvalidCoordinatesStillHoldsTheKnownEndpoint() {
        Pose endpoint = new Pose(10, 20, 0.6);
        drive.aimHybridHold(endpoint, Double.NaN, 0, 42, AprilTagObservation.unavailable(42));

        assertHoldAt(10, 20, 0.6);
        assertFalse(drive.isHybridVisionAligned());
    }

    private void hybrid(AprilTagObservation observation) {
        drive.aimHybrid(0, 0, -0.7, false, 10, 10, 42, observation);
    }

    @Test
    public void aimBrakesRotationBeforeCrossingTheTarget() {
        localizer.velocity = new Velocity(0, 0, -1);
        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 2, 2, 1));

        assertTrue("Deve frear o giro horario que ja esta rapido perto do alvo",
                drivetrain.powers.turn() > 0);
    }

    @Test
    public void aimLimitsTurnPowerWithTheRealRobotGain() {
        drive = new PedroDrive(follower, null, Controller.proportional(3.985729017698461), clock::get);
        drive.aim(0.6, 0.4, 0, false, AprilTagObservation.detected(42, 30, 2, 1));

        assertTrue(Math.abs(drivetrain.powers.turn()) <= 0.35);
        assertPowers(0.6, -0.4, drivetrain.powers.turn());
    }

    @Test
    public void hybridCannotReportAlignmentWhileStillSpinningThroughTheCenter() {
        localizer.velocity = new Velocity(0, 0, 1);
        for (int i = 1; i <= 3; i++) hybrid(AprilTagObservation.detected(42, 0, 2, i));

        assertFalse(drive.isHybridVisionAligned());
        assertTrue("Deve frear mesmo com a imagem centralizada", drivetrain.powers.turn() < 0);
    }

    private void lockHybridVision() {
        for (int i = 1; i <= 3; i++) hybrid(AprilTagObservation.detected(42, 6, 1, i));
    }

    @Test
    public void delayedImageUsesInterpolatedCaptureHeadingRatherThanCurrentHeading() {
        drive.run(0, 0, 0, false);
        clock.set(100_000_000);
        localizer.setPose(new Pose(0, 0, Math.toRadians(-6)));
        // A velocidade mudou desde a captura: extrapolar apenas pela velocidade atual seria errado.
        localizer.velocity = new Velocity(0, 0, Math.toRadians(-5));

        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 6, 2, 1, 0.1));

        assertPowers(0, 0, PedroDrive.AIM_TURN_DAMPING * Math.toRadians(5));
    }

    @Test
    public void captureHeadingAcrossWraparoundCorrectsAnImageThatWasPreviouslyCentered() {
        localizer.setPose(new Pose(0, 0, Math.toRadians(179)));
        drive.run(0, 0, 0, false);
        clock.set(100_000_000);
        localizer.setPose(new Pose(0, 0, Math.toRadians(-179)));

        drive.aim(0, 0, 0, false, AprilTagObservation.detected(42, 0, 2, 1, 0.1));

        assertPowers(0, 0, Math.toRadians(-2));
    }

    @Test
    public void delayedCameraAndRotationalInertiaSettleWithoutRepeatedOvershoot() {
        drive = new PedroDrive(follower, null, Controller.proportional(3.985729017698461), clock::get);
        double heading = Math.toRadians(-30), omega = 0;
        double maxOvershoot = 0;
        java.util.List<Double> headings = new java.util.ArrayList<>();
        AprilTagObservation frame = null;
        // Modelo: 20 ms/ciclo, camera a 25 Hz com 100 ms de atraso, inercia de 150 ms.
        for (int i = 0; i < 250; i++) {
            clock.set(i * 20_000_000L);
            localizer.setPose(new Pose(0, 0, heading));
            localizer.velocity = new Velocity(0, 0, omega);
            headings.add(heading);
            if (i % 2 == 0) {
                frame = AprilTagObservation.detected(42,
                        Math.toDegrees(headings.get(Math.max(0, i - 5))), 2, i, 0.1);
            }
            drive.aim(0, 0, 0, false, frame);
            omega += (3 * drivetrain.powers.turn() - omega) * 0.02 / 0.15;
            heading += omega * 0.02;
            maxOvershoot = Math.max(maxOvershoot, heading);
        }
        assertTrue("Overshoot em graus: " + Math.toDegrees(maxOvershoot),
                maxOvershoot < Math.toRadians(3));
        assertTrue("Erro final em graus: " + Math.toDegrees(heading),
                Math.abs(heading) <= Math.toRadians(1.2));
        assertTrue("Deve terminar com giro baixo", Math.abs(omega) < Math.toRadians(2));
    }

    private void assertHoldAt(double x, double y, double heading) {
        assertTrue(follower.holding());
        Pose target = follower.poseAt(0);
        assertEquals(x, target.x(), EPSILON);
        assertEquals(y, target.y(), EPSILON);
        double difference = target.heading() - heading;
        assertEquals(0, Math.atan2(Math.sin(difference), Math.cos(difference)), EPSILON);
    }

    private void assertPowers(double forward, double strafe, double turn) {
        assertEquals(forward, drivetrain.powers.forward(), EPSILON);
        assertEquals(strafe, drivetrain.powers.strafe(), EPSILON);
        assertEquals(turn, drivetrain.powers.turn(), EPSILON);
    }

    private static final class SimulatedLocalizer implements Localizer {
        private Pose pose = Pose.zero();
        private Velocity velocity = Velocity.zero();

        @Override
        public void setPose(Pose pose) {
            this.pose = pose;
        }

        @Override
        public MotionState state() {
            return MotionState.ofVelocity(pose, velocity);
        }

        @Override
        public void update() {}

        @Override
        public void reset() {
            pose = Pose.zero();
            velocity = Velocity.zero();
        }
    }

    private static final class RecordingDrivetrain implements Drivetrain {
        private DrivePowers powers = DrivePowers.zero();
        private boolean manual;

        @Override
        public void drive(DrivePowers powers, boolean manual) {
            this.powers = powers;
            this.manual = manual;
        }

        @Override
        public void stop() {
            powers = DrivePowers.zero();
        }

        @Override
        public void stop(boolean brake) {
            stop();
        }

        @Override
        public double maxScaling(DrivePowers current, DrivePowers delta) {
            return 1;
        }

        @Override
        public double interpolateVelocity(double xRadius, double yRadius, double theta) {
            return 1;
        }

        @Override
        public Map<String, Object> debug() {
            return Collections.emptyMap();
        }
    }
}
