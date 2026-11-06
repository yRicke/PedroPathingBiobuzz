package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced;

import org.junit.Test;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import static org.mockito.Mockito.*;
import java.util.OptionalDouble;
import java.util.concurrent.atomic.AtomicLong;
import static org.junit.Assert.*;

public class FlywheelAutomaticTest {
    private final AtomicLong clock = new AtomicLong();
    private final DcMotorEx motorOne = mock(DcMotorEx.class);
    private final DcMotorEx motorTwo = mock(DcMotorEx.class);
    private final Flywheel flywheel = new Flywheel(motorOne, motorTwo, null,
            new FlywheelRpmTable(2560, 2680, new double[][] {{100, 2560}, {200, 2680}}),
            clock::get);

    @Test public void firstDistanceIsAppliedImmediately() {
        assertEquals(2680, apply(OptionalDouble.of(2)), 0);
    }

    @Test public void holdsSmallNoiseAndRateLimitsLargerChanges() {
        assertEquals(2620, apply(OptionalDouble.of(1.5)), 1e-9);
        clock.set(100_000_000);
        assertEquals(2620, apply(OptionalDouble.of(1.51)), 1e-9);
        assertEquals(2632, apply(OptionalDouble.of(1.6)), 1e-9);
        clock.set(199_000_000);
        assertEquals(2632, apply(OptionalDouble.of(1.8)), 1e-9);
        clock.set(200_000_000);
        assertEquals(2656, apply(OptionalDouble.of(1.8)), 1e-9);
    }

    @Test public void lostDistanceImmediatelyReturnsToBaseAndResetStartsFresh() {
        assertEquals(2680, apply(OptionalDouble.of(2)), 0);
        clock.set(1);
        assertEquals(2560, apply(OptionalDouble.empty()), 0);
        flywheel.stopMotors();
        assertEquals(2680, apply(OptionalDouble.of(2)), 0);
    }

    @Test public void manualRpmBypassesDeadbandAndRestartsAutomaticMode() {
        assertEquals(2680, apply(OptionalDouble.of(2)), 0);
        clock.set(1);
        flywheel.run(2600.1);
        assertEquals(2600.1, flywheel.getTargetRPM(), 1e-9);
        clock.set(2);
        assertEquals(2680, apply(OptionalDouble.of(2)), 0);
    }

    @Test public void motorFeedbackRunsWhileAutomaticSetpointIsHeld() {
        apply(OptionalDouble.of(1.5));
        clearInvocations(motorOne, motorTwo);
        when(motorOne.getVelocity()).thenReturn(2620 * Flywheel.TICKS_PER_REVOLUTION / 60);
        clock.set(1);
        apply(OptionalDouble.of(1.501));
        assertEquals(2620, flywheel.getTargetRPM(), 1e-9);
        double feedforward = Flywheel.KS + Flywheel.KV * 2620;
        verify(motorOne).setPower(org.mockito.ArgumentMatchers.doubleThat(p -> Math.abs(p - feedforward) < 1e-9));
        verify(motorTwo).setPower(org.mockito.ArgumentMatchers.doubleThat(p -> Math.abs(p + feedforward) < 1e-9));
    }

    @Test public void automaticTargetsAreBoundedEvenWithAnOutOfRangeInjectedTable() {
        Flywheel bounded = new Flywheel(motorOne, motorTwo, null,
                new FlywheelRpmTable(2000, 6000, new double[][] {{100, 2000}, {200, 4000}}),
                clock::get);
        bounded.runByDistance(OptionalDouble.of(0.5));
        assertEquals(Flywheel.MIN_RPM, bounded.getTargetRPM(), 0);
        clock.set(100_000_000);
        bounded.runByDistance(OptionalDouble.of(3));
        assertEquals(Flywheel.MAX_RPM, bounded.getTargetRPM(), 0);
        bounded.runByDistance(OptionalDouble.empty());
        assertEquals(Flywheel.MIN_RPM, bounded.getTargetRPM(), 0);
        bounded.stopMotors();
        assertEquals(0, bounded.getTargetRPM(), 0);
    }

    private double apply(OptionalDouble distance) {
        flywheel.runByDistance(distance);
        return flywheel.getTargetRPM();
    }
}
