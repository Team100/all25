package org.team100.lib.trajectory;

import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.XYSeries;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.trajectory.timing.TimedState;

import edu.wpi.first.math.geometry.Rotation2d;

public class TrajectoryToVectorSeries {
    private static final boolean DEBUG = false;

    private static final int POINTS = 20;
    /** Length of the vector indicating heading */
    private final double m_scale;

    public TrajectoryToVectorSeries(double scale) {
        m_scale = scale;
    }

    /** Maps x to x, y to y */
    public VectorSeries convert(String name, Trajectory100 t) {
        VectorSeries s = new VectorSeries(name);
        double duration = t.duration();
        if (DEBUG)
            System.out.printf("duration %f\n", duration);
        double dt = duration / POINTS;
        for (double time = 0; time < duration; time += dt) {
            TimedState p = t.sample(time);
            WaypointSE2 pp = p.state().waypoint();
            double x = pp.pose().getTranslation().getX();
            double y = pp.pose().getTranslation().getY();
            Rotation2d heading = pp.pose().getRotation();
            double dx = m_scale * heading.getCos();
            double dy = m_scale * heading.getSin();
            s.add(x, y, dx, dy);
            if (DEBUG)
                System.out.printf("%f\n", time);
        }
        return s;
    }

    /**
     * X as a function of t.
     * 
     * @return (t, x)
     */
    public static XYSeries x(String name, Trajectory100 trajectory) {
        XYSeries series = new XYSeries(name);
        double duration = trajectory.duration();
        double dt = duration / POINTS;
        for (double t = 0; t <= duration + 0.0001; t += dt) {
            TimedState p = trajectory.sample(t);
            WaypointSE2 pp = p.state().waypoint();
            double x = pp.pose().getTranslation().getX();
            series.add(t, x);
        }
        return series;
    }

    /**
     * X dot: dx/dt, as a function of t.
     * 
     * @return (t, \dot{x})
     */
    public static XYSeries xdot(String name, Trajectory100 trajectory) {
        XYSeries series = new XYSeries(name);
        double duration = trajectory.duration();
        double dt = duration / POINTS;
        for (double t = 0; t <= duration + 0.0001; t += dt) {
            TimedState p = trajectory.sample(t);
            Rotation2d course = p.state().waypoint().course().toRotation();
            double velocityM_s = p.velocityM_S();
            System.out.println(velocityM_s);
            System.out.println(course);
            double xv = course.getCos() * velocityM_s;
            series.add(t, xv);
        }
        return series;
    }
}
