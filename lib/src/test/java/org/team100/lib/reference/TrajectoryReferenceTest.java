package org.team100.lib.reference;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;
import org.team100.lib.timing.TimingConstraint;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryMaker;

import edu.wpi.first.math.geometry.Pose2d;

public class TrajectoryReferenceTest implements Timeless {
    private static final double kDelta = 0.001;
    SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.get();
    List<TimingConstraint> constraints = new TimingConstraintFactory(swerveKinodynamics).allGood();
    TrajectoryMaker maker = new TrajectoryMaker(constraints);

    @Test
    void testSimple() {
        Trajectory100 t = maker.restToRest(
                new Pose2d(0, 0, GeometryUtil.kRotationZero),
                new Pose2d(1, 0, GeometryUtil.kRotationZero));
        TrajectoryReference r = new TrajectoryReference(t);
        // measurement is irrelevant
        r.initialize(new SwerveModel());
        {
            SwerveModel c = r.current();
            assertEquals(0, c.velocity().x(), kDelta);
            assertEquals(0, c.pose().getX(), kDelta);
            SwerveModel n = r.next();
            assertEquals(0.098, n.velocity().x(), kDelta);
            assertEquals(0, n.pose().getX(), kDelta);
        }
        // no time step, nothing changes
        {
            SwerveModel c = r.current();
            assertEquals(0, c.velocity().x(), kDelta);
            assertEquals(0, c.pose().getX(), kDelta);
            SwerveModel n = r.next();
            assertEquals(0.098, n.velocity().x(), kDelta);
            // x is very small but not zero
            assertEquals(0.00098, n.pose().getX(), 0.0000001);
        }
        // stepping time gets the next references
        stepTime();
        {
            SwerveModel c = r.current();
            assertEquals(0.098, c.velocity().x(), kDelta);
            assertEquals(0, c.pose().getX(), kDelta);
            SwerveModel n = r.next();
            assertEquals(0.196, n.velocity().x(), kDelta);
            assertEquals(0.004, n.pose().getX(), kDelta);
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
