package org.firstinspires.ftc.teamcode.subsystems.FlywheelAdvanced;

import org.junit.Test;
import java.util.OptionalDouble;
import java.util.concurrent.atomic.AtomicLong;
import static org.junit.Assert.*;

public class FlywheelRpmControllerTest {
    private final AtomicLong clock = new AtomicLong();
    private final FlywheelRpmController controller = new FlywheelRpmController(
            new FlywheelRpmTable(2720, 6000, new double[][] {{100, 2720}, {200, 3200}}),
            null, clock::get);

    @Test public void firstDistanceIsAppliedImmediately() {
        assertEquals(3200, controller.run(OptionalDouble.of(2)), 0);
    }

    @Test public void holdsSmallNoiseAndRateLimitsLargerChanges() {
        assertEquals(2960, controller.run(OptionalDouble.of(1.5)), 1e-9);
        clock.set(100_000_000);
        assertEquals(2960, controller.run(OptionalDouble.of(1.51)), 1e-9);
        assertEquals(2974.4, controller.run(OptionalDouble.of(1.53)), 1e-9);
        clock.set(199_000_000);
        assertEquals(2974.4, controller.run(OptionalDouble.of(1.6)), 1e-9);
        clock.set(200_000_000);
        assertEquals(3008, controller.run(OptionalDouble.of(1.6)), 1e-9);
    }

    @Test public void lostDistanceImmediatelyReturnsToBaseAndResetStartsFresh() {
        assertEquals(3200, controller.run(OptionalDouble.of(2)), 0);
        clock.set(1);
        assertEquals(2720, controller.run(OptionalDouble.empty()), 0);
        controller.reset();
        assertEquals(3200, controller.run(OptionalDouble.of(2)), 0);
    }
}
