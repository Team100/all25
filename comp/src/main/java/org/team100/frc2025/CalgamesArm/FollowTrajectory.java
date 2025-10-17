package org.team100.frc2025.CalgamesArm;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.reference.TrajectoryReferenceR3;
import org.team100.lib.trajectory.Trajectory100;

/** Analogous to DriveWithTrajectory, but for R3 positional control. */
public class FollowTrajectory extends MoveAndHold {

    private final CalgamesMech m_subsystem;
    private final Trajectory100 m_trajectory;

    private CalgamesReferenceController m_referenceController;

    public FollowTrajectory(
            CalgamesMech subsystem,
            Trajectory100 trajectory) {
        m_subsystem = subsystem;
        m_trajectory = trajectory;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_referenceController = new CalgamesReferenceController(
                m_subsystem,
                new TrajectoryReferenceR3(m_trajectory));
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    public boolean isDone() {
        return m_referenceController.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stop();
    }

}
