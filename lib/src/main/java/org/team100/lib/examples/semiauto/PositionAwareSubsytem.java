package org.team100.lib.examples.semiauto;

import java.util.function.Supplier;

import org.team100.lib.dashboard.Glassy;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.MockAngularPositionServo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Illustrates a self-aiming shooter angle mechanism.
 */
public class PositionAwareSubsytem extends SubsystemBase implements Glassy {
    private final Translation2d m_aimingPoint;
    private final Supplier<Pose2d> m_pose;
    private final AngularPositionServo m_servo;
    // key: range in meters. value: angle in radians
    private final InterpolatingDoubleTreeMap m_elevation;

    public PositionAwareSubsytem(Translation2d aimingPoint, Supplier<Pose2d> pose) {
        m_aimingPoint = aimingPoint;
        m_pose = pose;
        // see lib/examples/motion for a real subsystem example.
        m_servo = new MockAngularPositionServo();
        m_elevation = new InterpolatingDoubleTreeMap();
        loadTable();
    }

    public Command holdAimingPoint() {
        return run(() -> {
            Pose2d pose = m_pose.get();
            double range = pose.getTranslation().getDistance(m_aimingPoint);
            double elevation = m_elevation.get(range);
            m_servo.setPositionProfiled(elevation, 0);
        });
    }

    /**
     * These are values from the 2024 shooter, found through experiment.
     */
    private void loadTable() {
        m_elevation.put(1.49, 0.9);
        m_elevation.put(2.07, 0.78);
        m_elevation.put(2.5, 0.66);
        m_elevation.put(3.02, 0.59);
        m_elevation.put(3.59, 0.53);
        m_elevation.put(4.1, 0.475);
        m_elevation.put(4.5, 0.44);
    }

}
