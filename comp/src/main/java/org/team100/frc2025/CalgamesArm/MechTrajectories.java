package org.team100.frc2025.CalgamesArm;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.motion.kinematics.AnalyticalJacobian;
import org.team100.lib.motion.kinematics.ElevatorArmWristKinematics;
import org.team100.lib.motion.kinematics.JointAccelerations;
import org.team100.lib.motion.kinematics.JointVelocities;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.JointConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TorqueConstraint;
import org.team100.lib.trajectory.timing.YawRateConstraint;

import edu.wpi.first.wpilibj2.command.Command;

/** Make a trajectory from the start to the end and follow it. */
public class MechTrajectories extends Command {
    private static final boolean USE_JOINT_CONSTRAINT = false;
    private final CalgamesMech m_subsystem;
    private final TrajectoryPlanner m_planner;

    public MechTrajectories(
            CalgamesMech mech,
            ElevatorArmWristKinematics k,
            AnalyticalJacobian j) {
        m_subsystem = mech;
        List<TimingConstraint> c = new ArrayList<>();
        if (USE_JOINT_CONSTRAINT) {
            // This is experimental, don't use it.
            c.add(new JointConstraint(
                    k,
                    j,
                    new JointVelocities(10, 10, 10),
                    new JointAccelerations(10, 10, 10)));

        } else {
            // These are known to work, but suboptimal.
            c.add(new ConstantConstraint(10, 5));
            c.add(new YawRateConstraint(10, 5));
            // This is new
            c.add(new TorqueConstraint(20));
        }

        // ALERT!
        // The parameters here used to be double these values;
        // These finer grains make smoother paths and schedules but
        // take longer to compute, so if it takes too long, make these
        // numbers bigger!
        m_planner = new TrajectoryPlanner(0.01, 0.1, 0.05, c);
        // m_planner = new TrajectoryPlanner(0.02, 0.2, 0.1, c);
    }

    /** A command that goes from the start to the end and then finishes. */
    public Command terminal(String name, HolonomicPose2d start, HolonomicPose2d end) {

        /** Use the start course and ignore the start pose for now */
        MoveAndHold f = new GoToPoseCalGamesMech(m_subsystem, start.course(), end, m_planner);
        return f
                .until(f::isDone)
                .withName(name);
    }

    /** A command that goes from the start to the end and then waits forever. */
    public MoveAndHold endless(String name, HolonomicPose2d start, HolonomicPose2d end) {

        /** Use the start course and ignore the start pose for now */
        GoToPoseCalGamesMech c = new GoToPoseCalGamesMech(
                m_subsystem, start.course(), end, m_planner);
        c.setName(name);
        return c;

    }

}
