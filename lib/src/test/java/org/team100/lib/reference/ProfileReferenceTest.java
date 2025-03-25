package org.team100.lib.reference;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ProfileReferenceTest implements Timeless {
    private static final double kDelta = 0.001;

    @Test
    void testSimple() {
        SwerveModel measurement = new SwerveModel(new Pose2d(0, 0, Rotation2d.kZero));
        SwerveModel goal = new SwerveModel(new Pose2d(1, 0, Rotation2d.kZero));
        HolonomicProfile hp = HolonomicProfile.trapezoidal(1, 1, 0.01, 1, 1, 0.01);
        ProfileReference r = new ProfileReference(hp);
        r.setGoal(goal);
        r.initialize(measurement);
        {
            SwerveModel c = r.current();
            assertEquals(0, c.velocity().x(), kDelta);
            assertEquals(0, c.pose().getX(), kDelta);
            SwerveModel n = r.next();
            assertEquals(0.02, n.velocity().x(), kDelta);
            assertEquals(0, n.pose().getX(), kDelta);
        }
        // no time step, nothing changes
        {
            SwerveModel c = r.current();
            assertEquals(0, c.velocity().x(), kDelta);
            assertEquals(0, c.pose().getX(), kDelta);
            SwerveModel n = r.next();
            assertEquals(0.02, n.velocity().x(), kDelta);
            // x is very small but not zero
            assertEquals(0.0002, n.pose().getX(), 0.00001);
        }
        // stepping time gets the next references
        stepTime();
        {
            SwerveModel c = r.current();
            assertEquals(0.02, c.velocity().x(), kDelta);
            assertEquals(0, c.pose().getX(), kDelta);
            SwerveModel n = r.next();
            assertEquals(0.04, n.velocity().x(), kDelta);
            assertEquals(0.00078, n.pose().getX(), kDelta);
        }
        // way in the future, we're at the end.
        for (int i = 0; i < 500; ++i) {
            stepTime();
        }
        {
            SwerveModel c = r.current();
            assertEquals(0, c.velocity().x(), kDelta);
            assertEquals(1, c.pose().getX(), kDelta);
            SwerveModel n = r.next();
            assertEquals(0, n.velocity().x(), kDelta);
            assertEquals(1, n.pose().getX(), kDelta);
        }
    }

}
