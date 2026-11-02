package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.teamcode.OpModeStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.mockito.InjectMocks;
import org.mockito.Spy;
import org.mockito.MockitoAnnotations;

import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.CALLS_REAL_METHODS;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.doThrow;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

public class ExampleAutoPoseTest {
    private Pose measuredPose = Pose.zero();
    private final Localizer localizer = mock(Localizer.class, CALLS_REAL_METHODS);
    private final Drivetrain drivetrain = mock(Drivetrain.class);
    @Spy private Follower follower = new Follower(localizer, drivetrain,
            new Foresight(Constants.foresightConfig));
    @InjectMocks private ExampleAuto auto;
    private AutoCloseable mocks;

    @Before
    public void setUp() {
        OpModeStorage.clearAutonomousPose();
        mocks = MockitoAnnotations.openMocks(this);
        when(localizer.state()).thenAnswer(invocation ->
                MotionState.ofVelocity(measuredPose, Velocity.zero()));
    }

    @After
    public void tearDown() throws Exception {
        OpModeStorage.clearAutonomousPose();
        mocks.close();
    }

    @Test
    public void stopRefreshesSensorsAndPassesMeasuredPoseToTeleOp() {
        Pose stoppedMidPath = new Pose(30, 25, Math.toRadians(23));
        doAnswer(invocation -> {
            measuredPose = stoppedMidPath;
            return null;
        }).when(localizer).update();
        auto.start();

        auto.stop();

        assertSame(stoppedMidPath, OpModeStorage.consumeAutonomousPose());
        verify(localizer).update();
        verify(drivetrain).stop();
        assertTrue(follower.idle());
    }

    @Test
    public void cancelledBeforePlayDoesNotPublishAnAutonomousPose() {
        auto.stop();

        assertNull(OpModeStorage.consumeAutonomousPose());
        verify(localizer, never()).update();
        verify(drivetrain).stop();
    }

    @Test
    public void localizationFailureStillStopsDriveAndDoesNotPublishPose() {
        follower.manual(new DrivePowers(1, 0, 0));
        doThrow(new IllegalStateException("Sensor indisponivel")).when(localizer).update();
        auto.start();

        assertThrows(IllegalStateException.class, () -> auto.stop());

        assertTrue(follower.idle());
        verify(drivetrain).stop();
        assertNull(OpModeStorage.consumeAutonomousPose());
    }
}
