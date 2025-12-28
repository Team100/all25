package org.team100.lib.visualization;

import java.util.List;

import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.timing.TimedState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;

public class TrajectoryVisualization {
    private static final String TRAJECTORY = "trajectory";

    private final DoubleArrayLogger m_log_trajectory;

    public TrajectoryVisualization(LoggerFactory fieldLogger) {
        m_log_trajectory = fieldLogger.doubleArrayLogger(Level.TRACE, TRAJECTORY);
    }

    public void setViz(Trajectory100 trajectory) {
        if (trajectory == null)
            return;
        m_log_trajectory.log(() -> fromTrajectory100(trajectory));
    }

    private static double[] fromTrajectory100(Trajectory100 m_trajectory) {
        double[] arr = new double[m_trajectory.length() * 3];
        int ndx = 0;
        for (TimedState p : m_trajectory.getPoints()) {
            WaypointSE2 pose = p.point().waypoint();
            arr[ndx + 0] = pose.pose().getTranslation().getX();
            arr[ndx + 1] = pose.pose().getTranslation().getY();
            arr[ndx + 2] = pose.pose().getRotation().getDegrees();
            ndx += 3;
        }
        return arr;
    }

    public void setViz(Trajectory m_trajectory) {
        m_log_trajectory.log(() -> fromWPITrajectory(m_trajectory));
    }

    private static double[] fromWPITrajectory(Trajectory m_trajectory) {
        double[] arr = new double[m_trajectory.getStates().size() * 3];
        int ndx = 0;
        for (State p : m_trajectory.getStates()) {
            Pose2d pose = p.poseMeters;
            arr[ndx + 0] = pose.getTranslation().getX();
            arr[ndx + 1] = pose.getTranslation().getY();
            arr[ndx + 2] = pose.getRotation().getDegrees();
            ndx += 3;
        }
        return arr;
    }

    public void setViz(List<Pose2d> poses) {
        m_log_trajectory.log(() -> fromPoses(poses));
    }

    private static double[] fromPoses(List<Pose2d> poses) {
        double[] arr = new double[poses.size() * 3];
        int ndx = 0;
        for (Pose2d pose : poses) {
            arr[ndx + 0] = pose.getTranslation().getX();
            arr[ndx + 1] = pose.getTranslation().getY();
            arr[ndx + 2] = pose.getRotation().getDegrees();
            ndx += 3;
        }
        return arr;
    }

    public void clear() {
        m_log_trajectory.log(() -> new double[0]);
    }

}
