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
        verify(two).setDirection(DcMotorSimple.Direction.REVERSE);
        assertPower(0);
    }

    @Test public void spinupSaturatesWithoutRequestingMoreThanFullPower() {
        flywheel.run(2720);
        assertPower(1);
    }

    @Test public void atTargetUsesFeedforwardAndOverspeedNeverReverses() {
        measuredRpm(2620);
        flywheel.run(2620);
        assertPower(Flywheel.KS + Flywheel.KV * 2620);
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
        assertEquals(Flywheel.MAX_RPM, flywheel.getTargetRPM(), 0);
        assertPower(Math.min(1, Flywheel.KS + Flywheel.KV * Flywheel.MAX_RPM));
    }

    @Test public void manualTargetsCanGoBelowAndAboveAutomaticBounds() {
        for (double rpm : new double[] {1, 2200, 2800, 4000, 6000}) {
            flywheel.run(rpm);
            assertEquals(rpm, flywheel.getTargetRPM(), 0);
        }
        flywheel.run(0);
        assertEquals(0, flywheel.getTargetRPM(), 0);
    }

    @Test public void switchingBetweenManualAndAutomaticUsesTheCorrectBounds() {
        flywheel.run(4000);
        flywheel.runByDistance(java.util.OptionalDouble.of(4));
        assertEquals(Flywheel.AUTO_MAX_RPM, flywheel.getTargetRPM(), 0);
        flywheel.run(2200);
        assertEquals(2200, flywheel.getTargetRPM(), 0);
        flywheel.runByDistance(java.util.OptionalDouble.of(1));
        assertEquals(Flywheel.AUTO_MIN_RPM, flywheel.getTargetRPM(), 0);
    }

    @Test public void limitsFollowAllBaseRpmsAndCurrentCalibrationIsPreserved() {
        assertEquals(Math.min(Flywheel.BASE_RPM, Math.min(Flywheel.BASE_RPM_2, Flywheel.BASE_RPM_3)),
                Flywheel.AUTO_MIN_RPM, 0);
        assertEquals(Math.max(Flywheel.BASE_RPM, Math.max(Flywheel.BASE_RPM_2, Flywheel.BASE_RPM_3)),
                Flywheel.AUTO_MAX_RPM, 0);
        for (double[] pair : new double[][] {{1.0, 2560}, {1.82, 2560}, {1.85, 2570},
                {1.95, 2625}, {2.05, 2680}, {4.0, 2680}}) {
            flywheel.stopMotors();
            flywheel.runByDistance(java.util.OptionalDouble.of(pair[0]));
            assertEquals(pair[1], flywheel.getTargetRPM(), 1e-9);
        }
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
