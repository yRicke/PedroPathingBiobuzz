package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Limelight.Limelight;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class TeleOpMainTest {
    @Test
    public void leftHiveCoordinatesStayTheSameWhenAllianceChanges() {
        assertTarget(57.8, 90, TeleOpMain.targetForCell(Limelight.BLUE_AUDIENCE_FIRST_TAG));
        assertTarget(57.8, 90, TeleOpMain.targetForCell(Limelight.RED_AUDIENCE_FIRST_TAG));
    }

    @Test
    public void rightHiveCoordinatesStayTheSameWhenAllianceChanges() {
        assertTarget(57.8, 50, TeleOpMain.targetForCell(Limelight.BLUE_OPPOSITE_FIRST_TAG));
        assertTarget(57.8, 50, TeleOpMain.targetForCell(Limelight.RED_OPPOSITE_FIRST_TAG));
    }

    private static void assertTarget(double x, double y, Pose target) {
        assertEquals(x, target.x(), 1e-9);
        assertEquals(y, target.y(), 1e-9);
    }
}
