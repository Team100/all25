package org.team100.lib.trajectory;

import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.XYSeries;
import org.team100.lib.geometry.PathPoint;
import org.team100.lib.trajectory.path.Path100;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PathToVectorSeries {
    private static final boolean DEBUG = false;
    /** Length of the vector indicating heading */
    private final double m_scale;

    public PathToVectorSeries(double scale) {
        m_scale = scale;
    }

    /** Maps x to x, y to y */
    public VectorSeries convert(String name, Path100 path) {
        VectorSeries s = new VectorSeries(name);
        double l = path.getMaxDistance();
        double dl = l / 20;
        for (double d = 0; d < l; d += dl) {
            if (DEBUG)
                System.out.printf("%f\n", d);
            PathPoint pwm;
            pwm = path.sample(d);
            Pose2d p = pwm.waypoint().pose();
            double x = p.getX();
            double y = p.getY();
            Rotation2d heading = p.getRotation();
            double dx = m_scale * heading.getCos();
            double dy = m_scale * heading.getSin();
            s.add(x, y, dx, dy);
        }
        return s;
    }

    public static XYSeries x(String name, List<PathPoint> poses) {
        XYSeries series = new XYSeries(name);
        for (int i = 0; i < poses.size(); ++i) {
            PathPoint pose = poses.get(i);
            series.add(i, pose.waypoint().pose().getX());
        }
        return series;
    }

}
