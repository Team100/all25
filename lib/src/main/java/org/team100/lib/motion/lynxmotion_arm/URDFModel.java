package org.team100.lib.motion.lynxmotion_arm;

import java.util.List;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;

/**
 * This is a partial implementation of the URDF object model.
 * 
 * The idea is to use a more general mechanism representation than the usual
 * POJO (e.g. LynxArm, which uses fields), which would allow more general
 * kinematics than the hand-made stuff we've done previously (e.g. we could try
 * Cyclic Coordinate Descent, or Gradient Descent, in a general way)
 * 
 * It would be possible to serialize this to URDF XML and vice versa, but our
 * robots are small enough that (IMHO) putting the "loader" in code would be
 * much easier than bringing in the XML overhead.
 * 
 * See http://wiki.ros.org/urdf, in particular, urdf.xsd. I transcribed directly
 * from that file. (I tried to use JAXB to auto-generate this, but gave up
 * trying to run it.)
 * 
 * This omits the collision and visual parts of the model, which are used for
 * simulation and visualization, respectively. We could add those back in if we
 * need them someday. I haven't found the URDF visualizers (e.g.
 * https://mymodelrobot.appspot.com/5629499534213120) or simulators (e.g.
 * Gazebo) to be that useful.
 */
public interface URDFModel {

    public record Link(String name) {
    }

    public record Joint(
            String name,
            JointType type,
            Limit limit,
            Link parent,
            Link child,
            Pose3d origin,
            Vector<N3> axis) {
        public record Limit(double effort, double lower, double upper, double velocity) {
        }

        public enum JointType {
            revolute,
            continuous,
            prismatic,
            fixed,
            floating,
            planar,
        }

    }

    public record Robot(String name, List<Link> links, List<Joint> joints) {
    }

}
