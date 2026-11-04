package org.firstinspires.ftc.teamcode.subsystems.FlowerCollector;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Before;
import org.junit.Test;

import java.util.concurrent.atomic.AtomicLong;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class FlowerCollectorTest {
    private CRServo one, two;
    private FlowerCollector flowerCollector;
    private AtomicLong clock;
    private long durationNanos;

    @Before public void setup() {
        one = mock(CRServo.class);
        two = mock(CRServo.class);
        HardwareMap hardwareMap = mock(HardwareMap.class);
        when(hardwareMap.get(CRServo.class, "flowerCollect1")).thenReturn(one);
        when(hardwareMap.get(CRServo.class, "flowerCollect2")).thenReturn(two);
        clock = new AtomicLong(10_000_000_000L);
        durationNanos = (long) (FlowerCollector.SERVO_RUN_TIME_SECONDS * 1_000_000_000L);
        flowerCollector = new FlowerCollector(hardwareMap, mock(Telemetry.class), clock::get);
    }

    @Test public void startsStoppedWithBothServosInTheSameDirection() {
        verify(one).setDirection(DcMotorSimple.Direction.FORWARD);
        verify(two).setDirection(DcMotorSimple.Direction.FORWARD);
        assertPower(0);
        assertFalse(flowerCollector.isMoving());
        assertEquals(0, flowerCollector.getRemainingSeconds(), 0);
        clearInvocations(one, two);
        flowerCollector.update();
        verifyNoInteractions(one, two);
    }

    @Test public void openingRunsBothTogetherAndStopsExactlyAtTheDeadline() {
        clearInvocations(one, two);
        flowerCollector.open();
        assertPower(1);
        assertTrue(flowerCollector.isMoving());
        assertEquals(FlowerCollector.SERVO_RUN_TIME_SECONDS,
                flowerCollector.getRemainingSeconds(), 0);
        clearInvocations(one, two);
        clock.addAndGet(durationNanos - 1);
        flowerCollector.update();
        assertTrue(flowerCollector.isMoving());
        verifyNoInteractions(one, two);
        clock.incrementAndGet();
        flowerCollector.update();
        assertPower(0);
        assertFalse(flowerCollector.isMoving());
        assertEquals(0, flowerCollector.getRemainingSeconds(), 0);
    }

    @Test public void closingUsesTheOppositePowerForTheSameDuration() {
        clearInvocations(one, two);
        flowerCollector.close();
        assertPower(-1);
        assertTrue(flowerCollector.isMoving());
        clearInvocations(one, two);
        clock.addAndGet(durationNanos);
        flowerCollector.update();
        assertPower(0);
        assertFalse(flowerCollector.isMoving());
    }

    @Test public void repeatingOpenDoesNotExtendTheDeadlineOrRestartAfterCompletion() {
        flowerCollector.open();
        clearInvocations(one, two);
        clock.addAndGet(durationNanos / 2);
        for (int i = 0; i < 10; i++) flowerCollector.open();
        assertEquals(FlowerCollector.SERVO_RUN_TIME_SECONDS / 2,
                flowerCollector.getRemainingSeconds(), 1e-9);
        verifyNoInteractions(one, two);
        clock.addAndGet(durationNanos - durationNanos / 2);
        flowerCollector.open();
        assertPower(0);
        clearInvocations(one, two);
        for (int i = 0; i < 10; i++) flowerCollector.open();
        verifyNoInteractions(one, two);
        assertFalse(flowerCollector.isMoving());
    }

    @Test public void runDeduplicatesClosingAndCanOpenAfterClosingHasFinished() {
        flowerCollector.run(true);
        clearInvocations(one, two);
        clock.addAndGet(durationNanos);
        flowerCollector.run(true);
        assertPower(0);
        clearInvocations(one, two);
        flowerCollector.run(true);
        verifyNoInteractions(one, two);
        flowerCollector.run(false);
        assertPower(1);
        assertTrue(flowerCollector.isMoving());
    }

    @Test public void reversingMidMovementStartsAFullNewInterval() {
        flowerCollector.open();
        clock.addAndGet(durationNanos / 2);
        clearInvocations(one, two);
        flowerCollector.close();
        assertPower(-1);
        assertEquals(FlowerCollector.SERVO_RUN_TIME_SECONDS,
                flowerCollector.getRemainingSeconds(), 0);
        clearInvocations(one, two);
        clock.addAndGet(durationNanos / 2);
        flowerCollector.update();
        assertTrue(flowerCollector.isMoving());
        verifyNoInteractions(one, two);
        clock.addAndGet(durationNanos - durationNanos / 2);
        flowerCollector.update();
        assertPower(0);
        assertFalse(flowerCollector.isMoving());
    }

    @Test public void lateUpdateStopsBothWithoutReplayingAnyMovement() {
        flowerCollector.open();
        clearInvocations(one, two);
        clock.addAndGet(durationNanos * 3);
        assertEquals(0, flowerCollector.getRemainingSeconds(), 0);
        flowerCollector.update();
        assertPower(0);
        clearInvocations(one, two);
        flowerCollector.update();
        verifyNoInteractions(one, two);
        assertFalse(flowerCollector.isMoving());
    }

    @Test public void stopCancelsImmediatelyAndAllowsTheSameCommandAgain() {
        flowerCollector.open();
        clock.addAndGet(durationNanos / 2);
        clearInvocations(one, two);
        flowerCollector.stop();
        assertPower(0);
        assertFalse(flowerCollector.isMoving());
        clearInvocations(one, two);
        flowerCollector.open();
        assertPower(1);
        assertTrue(flowerCollector.isMoving());
        assertEquals(FlowerCollector.SERVO_RUN_TIME_SECONDS,
                flowerCollector.getRemainingSeconds(), 0);
    }

    private void assertPower(double expected) {
        verify(one).setPower(expected);
        verify(two).setPower(expected);
    }
}
