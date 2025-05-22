package org.team100.lib.motion.urdf;

import edu.wpi.first.math.geometry.Transform2d;

public class URDFUtil {
    public static Transform2d jointTransform(URDFModel.Joint joint) {

    }

    public static URDFModel.Joint getJoint(URDFModel.Robot robot, String name) {
        for (URDFModel.Joint joint : robot.joints()) {
            if (joint.name().equals(name))
                return joint;
        }
        return null;
    }
}
