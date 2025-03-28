package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class NearStationTest {
    @Test
    void testStationReg() {
        NearStation test = new NearStation(() -> new Pose2d());
        boolean works = test.closeToStation();
        assertTrue(works);
    }

    @Test
    void testStationBad() {
        NearStation test1 = new NearStation(() -> new Pose2d(4, 4, new Rotation2d()));
        boolean works1 = test1.closeToStation();
        assertFalse(works1);
    }
    @Test
    void testStationGood() {
        NearStation test2 = new NearStation(() -> new Pose2d(1, 7, new Rotation2d()));
        boolean works2= test2.closeToStation();
        assertTrue(works2);
    }
}
