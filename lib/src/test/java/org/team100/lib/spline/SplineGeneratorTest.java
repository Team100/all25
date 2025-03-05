package org.team100.lib.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

class SplineGeneratorTest {
    @Test
    void test() {
        HolonomicPose2d p1 = new HolonomicPose2d(new Translation2d(0, 0), Rotation2d.kZero, Rotation2d.kZero);
        HolonomicPose2d p2 = new HolonomicPose2d(new Translation2d(15, 10), Rotation2d.kZero, new Rotation2d(1, 5));
        HolonomicSpline s = new HolonomicSpline(p1, p2);

        List<Pose2dWithMotion> samples = SplineGenerator.parameterizeSpline(s, 0.05, 0.05, 0.1, 0.0, 1.0);

        double arclength = 0;
        Pose2dWithMotion cur_pose = samples.get(0);
        for (Pose2dWithMotion sample : samples) {
            Twist2d twist = GeometryUtil.slog(
                    GeometryUtil.transformBy(
                            GeometryUtil.inverse(cur_pose.getPose()), sample.getPose()));
            arclength += Math.hypot(twist.dx, twist.dy);
            cur_pose = sample;
        }

        assertEquals(15.0, cur_pose.getTranslation().getX(), 0.001);
        assertEquals(10.0, cur_pose.getTranslation().getY(), 0.001);
        assertEquals(78.690, cur_pose.getCourse().get().getDegrees(), 0.001);
        assertEquals(20.416, arclength, 0.001);
    }
}
