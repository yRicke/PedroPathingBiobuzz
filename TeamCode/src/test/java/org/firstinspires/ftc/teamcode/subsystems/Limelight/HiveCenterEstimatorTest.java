package org.firstinspires.ftc.teamcode.subsystems.Limelight;

import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.junit.Test;

import java.util.Arrays;
import java.util.Optional;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class HiveCenterEstimatorTest {
    private static final double EPSILON = 1e-9;

    @Test
    public void eachOfEightTagsAloneFindsSameCenterForItsHive() {
        int[] ids = {38, 39, 40, 41, 42, 43, 44, 45};
        double[] tagX = {0.1349, 0.23015, 0.36985, 0.4651,
                0.1349, 0.23015, 0.36985, 0.4651};

        for (int i = 0; i < ids.length; i++) {
            FiducialResult tag = tag(ids[i], tagX[i], -0.4, 2, 0, 0, 0, 1);
            int firstTag = ids[i] <= 41 ? 38 : 42;

            Optional<HiveCenterEstimator.Estimate> result =
                    HiveCenterEstimator.estimate(Arrays.asList(tag), firstTag);

            assertTrue("Tag " + ids[i], result.isPresent());
            assertEquals(8.530765609948133, result.get().txDegrees, EPSILON);
            assertEquals(1, result.get().tagsUsed);
        }
    }

    @Test
    public void hiddenRightmostTagDoesNotMoveEstimatedCenter() {
        Optional<HiveCenterEstimator.Estimate> result = HiveCenterEstimator.estimate(
                Arrays.asList(tag(42, -0.1651, 0, 2, 0, 0, 0, 1),
                        tag(43, -0.06985, 0, 2, 0, 0, 0, 1),
                        tag(44, 0.06985, 0, 2, 0, 0, 0, 1)), 42);

        assertTrue(result.isPresent());
        assertEquals(0, result.get().txDegrees, EPSILON);
        assertEquals(3, result.get().tagsUsed);
    }

    @Test
    public void areaWeightedFusionCombinesCenterPositionsRatherThanTagCenters() {
        Optional<HiveCenterEstimator.Estimate> result = HiveCenterEstimator.estimate(
                Arrays.asList(tag(42, -0.1651, 0, 2, 0, 0, 0, 1),
                        tag(45, 0.5651, 0, 2, 0, 0, 0, 3)), 42);

        assertTrue(result.isPresent());
        assertEquals(8.530765609948133, result.get().txDegrees, EPSILON);
        assertEquals(2, result.get().tagsUsed);
    }

    @Test
    public void tagRotationProjectsOffsetWithDepthAndHeightChanges() {
        FiducialResult tilted = tag(42, 0.2285096029175946, -0.441275,
                2.142980794164811, 17, 60, 30, 1);

        Optional<HiveCenterEstimator.Estimate> result =
                HiveCenterEstimator.estimate(Arrays.asList(tilted), 42);

        assertTrue(result.isPresent());
        assertEquals(8.530765609948133, result.get().txDegrees, EPSILON);
    }

    @Test
    public void upsideDownCameraPoseRotatesTheTagOffsetBeforeProjection() {
        FiducialResult rotated = tag(42, -0.1349, 0.4, 2, 0, 0, 180, 1);

        Optional<HiveCenterEstimator.Estimate> result =
                HiveCenterEstimator.estimate(Arrays.asList(rotated), 42);

        assertTrue(result.isPresent());
        assertEquals(-8.530765609948133, result.get().txDegrees, EPSILON);
    }

    @Test
    public void distanceUnitIsConvertedBeforeApplyingMetricGeometry() {
        FiducialResult inches = mock(FiducialResult.class);
        when(inches.getFiducialId()).thenReturn(42);
        when(inches.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                new Position(DistanceUnit.INCH, -6.5, 0, 40, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0)));

        Optional<HiveCenterEstimator.Estimate> result =
                HiveCenterEstimator.estimate(Arrays.asList(inches), 42);

        assertTrue(result.isPresent());
        assertEquals(0, result.get().txDegrees, EPSILON);
    }

    @Test
    public void otherHiveDoesNotContributeToEstimate() {
        assertFalse(HiveCenterEstimator.estimate(
                Arrays.asList(tag(38, 0, 0, 2, 0, 0, 0, 1)), 42).isPresent());
    }

    @Test
    public void missingZeroBehindOrNonFinitePoseCannotProduceHeading() {
        FiducialResult missing = mock(FiducialResult.class);
        when(missing.getFiducialId()).thenReturn(43);

        assertFalse(HiveCenterEstimator.estimate(Arrays.asList(missing,
                tag(43, 0, 0, 0, 0, 0, 0, 1),
                tag(43, 0, 0, -2, 0, 0, 0, 1),
                tag(43, Double.NaN, 0, 2, 0, 0, 0, 1),
                tag(43, 0, 0, 2, 0, Double.NaN, 0, 1)), 42).isPresent());
    }

    @Test
    public void invalidTagDoesNotDiscardAnotherUsableTag() {
        Optional<HiveCenterEstimator.Estimate> result = HiveCenterEstimator.estimate(
                Arrays.asList(tag(42, 0, 0, 0, 0, 0, 0, 1),
                        tag(43, -0.06985, 0, 2, 0, 0, 0, 1)), 42);

        assertTrue(result.isPresent());
        assertEquals(1, result.get().tagsUsed);
    }

    private FiducialResult tag(int id, double x, double y, double z,
                               double roll, double pitch, double yaw, double area) {
        FiducialResult tag = mock(FiducialResult.class);
        when(tag.getFiducialId()).thenReturn(id);
        when(tag.getTargetArea()).thenReturn(area);
        when(tag.getTargetPoseCameraSpace()).thenReturn(new Pose3D(
                new Position(DistanceUnit.METER, x, y, z, 0),
                new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, 0)));
        return tag;
    }
}
