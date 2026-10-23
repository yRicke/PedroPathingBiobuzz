package org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.pedropathing.algorithm.Foresight;
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
        drive = new PedroDrive(follower, null);
    }

    @Test
    public void forwardRemainsFieldRelativeAfterQuarterTurn() {
        localizer.setPose(new Pose(0, 0, Math.PI / 2));

        drive.run(1, 0, 0, false, 0);

        assertPowers(0, -1, 0);
        assertTrue(drivetrain.manual);
    }

    @Test
    public void rightStrafeAndClockwiseTurnKeepDriverDirections() {
        drive.run(0, 1, 0.5, false, 0);

        assertPowers(0, -1, -0.5);
    }

    @Test
    public void precisionScalesTranslationAndCombinedTurn() {
        drive.run(1, 0.5, 0.5, true, 0.5);

        assertPowers(0.3, -0.15, -0.195);
    }

    @Test
    public void secondDriverCanLeaveHoldInPrecisionMode() {
        drive.run(0, 0, 0, true, 0);
        assertTrue(follower.holding());

        drive.run(0, 0, 0, true, 1);

        assertTrue(follower.manual());
        assertPowers(0, 0, -0.09);
    }

    @Test
    public void releasedControlsHoldStoppedPoseUntilDrivingResumes() {
        Pose stoppedPose = new Pose(10, 20, 0.6);
        localizer.setPose(stoppedPose);
        localizer.velocity = new Velocity(1, 0, 0);
        drive.run(0.5, 0, 0, false, 0);

        drive.run(0, 0, 0, false, 0);
        assertTrue(follower.manual());
        assertPowers(0, 0, 0);

        localizer.velocity = Velocity.zero();
        drive.run(0, 0, 0, false, 0);
        assertTrue(follower.holding());
        assertSame(stoppedPose, follower.poseAt(0));

        localizer.setPose(new Pose(12, 19, 0.8));
        drive.run(0, 0, 0, false, 0);
        assertTrue(follower.holding());
        assertSame(stoppedPose, follower.poseAt(0));

        drive.run(0.5, 0, 0, false, 0);
        assertTrue(follower.manual());
        assertTrue(drivetrain.manual);
    }

    @Test
    public void stopImmediatelyRemovesMotorPower() {
        drive.run(1, 0, 0, false, 0);

        drive.stop();

        assertTrue(follower.idle());
        assertPowers(0, 0, 0);
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
