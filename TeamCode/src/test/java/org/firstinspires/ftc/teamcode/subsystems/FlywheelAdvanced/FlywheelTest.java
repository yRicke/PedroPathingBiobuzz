package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.junit.Before;
import org.junit.Test;
import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class FlywheelTest {
    private DcMotorEx one, two;
    private Flywheel flywheel;

    @Before public void setup() {
        one = mock(DcMotorEx.class);
        two = mock(DcMotorEx.class);
        flywheel = new Flywheel(one, two, null);
        clearInvocations(one, two);
    }

    @Test public void initializesKnownDirectionsAndStartsStopped() {
        flywheel = new Flywheel(one, two, null);
        verify(one).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(two).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verify(one).setDirection(DcMotorSimple.Direction.REVERSE);
        verify(two).setDirection(DcMotorSimple.Direction.FORWARD);
        assertPower(0);
    }

    @Test public void spinupSaturatesWithoutRequestingMoreThanFullPower() {
        flywheel.run(2720);
        assertPower(1);
    }

    @Test public void atTargetUsesFeedforwardAndOverspeedNeverReverses() {
        measuredRpm(2720);
        flywheel.run(2720);
        assertPower(Flywheel.KS + Flywheel.KV * 2720);
        clearInvocations(one, two);
        measuredRpm(3100);
        flywheel.run(2720);
        assertPower(0);
    }

    @Test public void invalidSetpointOrPrimaryEncoderStopsBothMotors() {
        for (double invalid : new double[] {0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
            clearInvocations(one, two);
            flywheel.run(invalid);
            assertPower(0);
        }
        clearInvocations(one, two);
        when(one.getVelocity()).thenReturn(Double.NaN);
        flywheel.run(2720);
        assertPower(0);
    }

    @Test public void targetIsBoundedToConfiguredRpmLimit() {
        measuredRpm(Flywheel.MAX_RPM);
        flywheel.run(Flywheel.MAX_RPM + 1000);
        assertPower(Flywheel.KS + Flywheel.KV * Flywheel.MAX_RPM);
    }

    @Test public void manualPowerAndTunerGainsUseTheSameLimits() {
        flywheel.setMotorPower(-0.2);
        assertPower(0);
        clearInvocations(one, two);
        flywheel.setGains(0.1, 0.0001, 0);
        flywheel.run(2000);
        assertPower(0.3);
        clearInvocations(one, two);
        assertThrows(IllegalArgumentException.class, () -> flywheel.setGains(0, Double.NaN, 0));
        assertPower(0);
    }

    @Test public void telemetryRpmsUseConsistentSignsAndUnitsForBothMotors() {
        measuredRpm(2720);
        when(two.getVelocity()).thenReturn(-2720 * Flywheel.TICKS_PER_REVOLUTION / 60);
        assertEquals(2720, flywheel.getRPM(), 1e-9);
        assertEquals(2720, flywheel.getSecondMotorRPM(), 1e-9);
    }

    private void measuredRpm(double rpm) {
        when(one.getVelocity()).thenReturn(rpm * Flywheel.TICKS_PER_REVOLUTION / 60);
    }
    private void assertPower(double expected) {
        verify(one).setPower(org.mockito.ArgumentMatchers.doubleThat(p -> Math.abs(p - expected) < 1e-9));
        verify(two).setPower(org.mockito.ArgumentMatchers.doubleThat(p -> Math.abs(p + expected) < 1e-9));
    }
}
