package org.firstinspires.ftc.teamcode.subsystems.FlowerCollect;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class FlowerCollectTest {
    private Servo one, two;
    private FlowerCollect flowerCollect;

    @Before public void setup() {
        one = mock(Servo.class);
        two = mock(Servo.class);
        HardwareMap hardwareMap = mock(HardwareMap.class);
        when(hardwareMap.get(Servo.class, "flowerCollect1")).thenReturn(one);
        when(hardwareMap.get(Servo.class, "flowerCollect2")).thenReturn(two);
        flowerCollect = new FlowerCollect(hardwareMap, mock(Telemetry.class));
    }

    @Test public void startsClosedWithOppositeDirections() {
        verify(one).setDirection(Servo.Direction.REVERSE);
        verify(two).setDirection(Servo.Direction.FORWARD);
        assertCommand(FlowerCollect.SERVO_ONE_CLOSED_POSITION,
                FlowerCollect.SERVO_TWO_CLOSED_POSITION);
    }

    @Test public void canStartClosedWithoutCommandingOpen() {
        HardwareMap hardwareMap = mock(HardwareMap.class);
        when(hardwareMap.get(Servo.class, "flowerCollect1")).thenReturn(one);
        when(hardwareMap.get(Servo.class, "flowerCollect2")).thenReturn(two);
        clearInvocations(one, two);

        flowerCollect = new FlowerCollect(hardwareMap, mock(Telemetry.class), true);

        assertCommand(FlowerCollect.SERVO_ONE_CLOSED_POSITION,
                FlowerCollect.SERVO_TWO_CLOSED_POSITION);
        verify(one, never()).setPosition(FlowerCollect.SERVO_ONE_OPEN_POSITION);
        verify(two, never()).setPosition(FlowerCollect.SERVO_TWO_OPEN_POSITION);
    }

    @Test public void independentPositionsDoNotDoubleReverseSecondServo() {
        clearInvocations(one, two);
        flowerCollect.setPositions(0.237, 0.412);
        assertCommand(0.237, 0.412);
    }

    @Test public void adjustingOneServoPreservesTheOtherPosition() {
        flowerCollect.setPositions(0.237, 0.412);
        clearInvocations(one, two);
        flowerCollect.setServoOnePosition(0.3);
        verify(one).setPosition(0.3);
        verifyNoInteractions(two);
        assertEquals(0.412, flowerCollect.getServoTwoPosition(), 0);
        clearInvocations(one, two);
        flowerCollect.setServoTwoPosition(0.5);
        verify(two).setPosition(0.5);
        verifyNoInteractions(one);
        assertEquals(0.3, flowerCollect.getServoOnePosition(), 0);
    }

    @Test public void positionsAreClampedToServoBounds() {
        clearInvocations(one, two);
        flowerCollect.setPositions(1.2, -0.2);
        assertCommand(1, 0);
        clearInvocations(one, two);
        flowerCollect.setPositions(-0.2, 1.2);
        assertCommand(0, 1);
    }

    @Test public void invalidPositionPreservesLastCommand() {
        flowerCollect.setPositions(0.237, 0.412);
        clearInvocations(one, two);
        for (double invalid : new double[] {Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY}) {
            assertThrows(IllegalArgumentException.class,
                    () -> flowerCollect.setPositions(invalid, 0.5));
            assertThrows(IllegalArgumentException.class,
                    () -> flowerCollect.setPositions(0.5, invalid));
        }
        assertEquals(0.237, flowerCollect.getServoOnePosition(), 0);
        assertEquals(0.412, flowerCollect.getServoTwoPosition(), 0);
        verifyNoInteractions(one, two);
    }

    @Test public void runSelectsOpenAndConfiguredClosedPositions() {
        clearInvocations(one, two);
        flowerCollect.run(true);
        assertCommand(FlowerCollect.SERVO_ONE_OPEN_POSITION,
                FlowerCollect.SERVO_TWO_OPEN_POSITION);
        clearInvocations(one, two);
        flowerCollect.run(false);
        assertCommand(FlowerCollect.SERVO_ONE_CLOSED_POSITION,
                FlowerCollect.SERVO_TWO_CLOSED_POSITION);
    }

    private void assertCommand(double expectedOne, double expectedTwo) {
        verify(one).setPosition(expectedOne);
        verify(two).setPosition(expectedTwo);
        assertEquals(expectedOne, flowerCollect.getServoOnePosition(), 0);
        assertEquals(expectedTwo, flowerCollect.getServoTwoPosition(), 0);
    }
}
