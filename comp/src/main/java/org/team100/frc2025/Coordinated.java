package org.team100.frc2025;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.motion.CoordinatedKinematics;
import org.team100.lib.motion.CoordinatedKinematics.Joints;
import org.team100.lib.util.Debug;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Coordinates the wrist and elevator to trace a simple 2d path.
 * 
 * As an example, the path is a square.
 */
public class Coordinated extends Command implements Debug {
    /** There are something like 20 sanjan units per meter? */
    private static final double sanjanunit = 20;
    /** Length of the arm. Not really 1 meter long, I guess? */
    private static final double LENGTH = 1;
    /** How long to take drawing each side. */
    private static final double DURATION = 1;
    /** Points to connect with straight lines. */
    private static final Translation2d[] POINTS = {
            new Translation2d(1, 0.5),
            new Translation2d(1.5, 0),
            new Translation2d(2, 0.5),
            new Translation2d(1.5, 1),
            new Translation2d(1, 0.5) };

    private final Elevator m_elevator;
    private final Wrist2 m_wrist;
    /** Timer for each side. */
    private final Takt.Timer m_timer;
    private final CoordinatedKinematics m_kinematics;
    /** Side index. */
    private int m_side;

    public Coordinated(Elevator elevator, Wrist2 wrist) {
        m_elevator = elevator;
        m_wrist = wrist;
        m_timer = new Takt.Timer();
        m_kinematics = new CoordinatedKinematics(LENGTH);
        addRequirements(elevator, wrist);
    }

    @Override
    public void initialize() {
        if (RobotBase.isReal()) {
            Util.warn("for simulation only");
            return;
        }
        m_timer.reset();
        m_side = 0;
    }

    @Override
    public void execute() {
        if (RobotBase.isReal()) {
            Util.warn("for simulation only");
            return;
        }

        if (m_timer.get() > DURATION) {
            m_timer.reset();
            m_side += 1;
            if (m_side >= POINTS.length - 1)
                m_side = 0;
        }
        Translation2d s0 = POINTS[m_side];
        Translation2d s1 = POINTS[m_side + 1];
        double fraction = m_timer.get() / DURATION;
        Translation2d setpoint = s0.interpolate(s1, fraction);
        Joints joints = m_kinematics.inverse(setpoint);
        if (joints == null) {
            // we tried to go outside the envelope
            cancel();
            return;
        }
        // these currently set goals, with profiles. we don't want that, we want to set
        // the setpoints.
        m_elevator.setPosition(sanjanunit * joints.height());
        m_wrist.setAngleValue(joints.angle());
        debug("%12.6f %12.6f\n", joints.height(), joints.angle());
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stop();
        m_wrist.stop();
    }

    @Override
    public boolean debug() {
        return false;
    }

}
