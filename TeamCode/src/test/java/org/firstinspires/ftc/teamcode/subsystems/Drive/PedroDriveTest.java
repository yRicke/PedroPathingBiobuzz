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

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight.AprilTagObservation;

import java.util.Collections;
import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

public class PedroDriveTest {
    private static final double EPSILON = 1e-9;

    private final SimulatedLocalizer localizer = new SimulatedLocalizer();
    private final RecordingDrivetrain drivetrain = new RecordingDrivetrain();
    private Follower follower;
    private PedroDrive drive;

    @Before
    public void setUp() {
        follower = new Follower(localizer, drivetrain, new Foresight(Constants.foresightConfig));
        drive = new PedroDrive(follower, null, Controller.proportional(1));
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
