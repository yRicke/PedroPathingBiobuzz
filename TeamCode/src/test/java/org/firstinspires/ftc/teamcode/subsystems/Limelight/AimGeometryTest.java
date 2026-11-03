package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;

public class AimGeometryTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void everyTagIndependentlyFindsTheOfficialOpeningOrigin() {
        double[] tagX = {-0.1651, -0.06985, 0.06985, 0.1651};
        for (int i = 0; i < tagX.length; i++) {
            // Abertura conhecida em (0.3, -0.4, 2); tags no plano oficial do SDK.
            AimGeometry.Point opening = AimGeometry.openingFromTag(i,
                    0.3 + tagX[i], -0.4 + 0.18255996, 2 - 0.1427988, 0, 0, 0);
            assertPoint(0.3, -0.4, 2, opening);
        }
    }

    @Test
    public void tiltedCellRotatesHeightAndDepthOffsetsTogether() {
        // R_x(90): offset (0.1651, -0.18255996, 0.1427988)
        // torna-se (0.1651, -0.1427988, -0.18255996).
        AimGeometry.Point opening = AimGeometry.openingFromTag(0,
                0.1349, -0.2572012, 2.18255996, Math.PI / 2, 0, 0);
        assertPoint(0.3, -0.4, 2, opening);
    }

    @Test
    public void invertedImageRotatesTheOpeningOffsetBeforeCameraCompensation() {
        AimGeometry.Point cameraOpening = AimGeometry.openingFromTag(0,
                -0.1349, 0.21744004, 1.8572012, 0, 0, Math.PI);
        AimGeometry geometry = new AimGeometry(0, 0, 0, 0, 0, 180);

        assertPoint(-0.3, 0.4, 2, cameraOpening);
        assertPoint(0.3, -0.4, 2, geometry.fromCamera(cameraOpening));
    }

    @Test
    public void cameraPositionIsAppliedFromTheBallExitRatherThanTheRobotCenter() {
        AimGeometry geometry = new AimGeometry(10, 20, -30, 0, 0, 0);

        assertPoint(0.4, -0.6, 1.7,
                geometry.fromCamera(new AimGeometry.Point(0.3, -0.4, 2)));
    }

    @Test
    public void cameraPitchIsRemovedBeforeComputingHorizontalAim() {
        AimGeometry geometry = new AimGeometry(0, 0, 0, 30, 0, 0);

        assertPoint(0.3, -1, Math.sqrt(3),
                geometry.fromCamera(new AimGeometry.Point(0.3, 0, 2)));
    }

    @Test
    public void invertedPitchedAndYawedCameraUsesTheMountRotationInOrder() {
        AimGeometry geometry = new AimGeometry(10, 20, -30, 30, 90, 180);
        // Roll: (-1,-2,3); pitch: (-1,-sqrt(3)-1.5,-1+1.5*sqrt(3));
        // yaw de 90 graus troca Z por X e -X por Z; translacao e aplicada ao final.
        assertPoint(-0.9 + 1.5 * Math.sqrt(3), -Math.sqrt(3) - 1.7, 0.7,
                geometry.fromCamera(new AimGeometry.Point(1, 2, 3)));
    }

    @Test
    public void lateralCameraOffsetRequiresDifferentAnglesAtDifferentDistances() {
        AimGeometry geometry = new AimGeometry(10, 0, 0, 0, 0, 0);
        AimGeometry.Point near = geometry.fromCamera(new AimGeometry.Point(0, 0, 1));
        AimGeometry.Point far = geometry.fromCamera(new AimGeometry.Point(0, 0, 3));

        assertEquals(5.710593137499643, Math.toDegrees(Math.atan2(near.x, near.z)), EPSILON);
        assertEquals(1.9091524329963763, Math.toDegrees(Math.atan2(far.x, far.z)), EPSILON);
    }

    @Test
    public void nonFiniteMountMeasurementsCannotBeTreatedAsARealCalibration() {
        assertThrows(IllegalArgumentException.class,
                () -> new AimGeometry(Double.NaN, 0, 0, 0, 0, 0));
        assertThrows(IllegalArgumentException.class,
                () -> new AimGeometry(0, 0, 0, Double.POSITIVE_INFINITY, 0, 0));
    }

    private static void assertPoint(double x, double y, double z, AimGeometry.Point point) {
        assertEquals(x, point.x, EPSILON);
        assertEquals(y, point.y, EPSILON);
        assertEquals(z, point.z, EPSILON);
    }
}
