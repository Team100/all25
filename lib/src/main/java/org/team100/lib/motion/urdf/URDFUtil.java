package org.team100.lib.motion.urdf;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Maybe some of this should go in the model itself. */
public class URDFUtil {
    /** All joint poses. */
    public static Map<String, Pose3d> forward(
            URDFModel.Robot robot,
            Map<String, Double> q) {
        Map<String, Pose3d> poses = new HashMap<>();
        for (URDFModel.Joint joint : robot.joints()) {
            forward(robot, poses, joint.name(), q);
        }
        return poses;
    }

    /**
     * Populate poses with the pose of the specified joint and those of its parent
     * chain.
     */
    static Pose3d forward(
            URDFModel.Robot robot,
            Map<String, Pose3d> poses,
            String jointName,
            Map<String, Double> q) {
        URDFModel.Joint joint = getJoint(robot, jointName);
        Transform3d t = jointTransform(joint, q.get(jointName));
        URDFModel.Joint parent = parentJoint(robot, jointName);
        if (parent == null) {
            // this is the root
            Pose3d basePose = Pose3d.kZero.transformBy(t);
            poses.put(jointName, basePose);
            return basePose;
        }
        Pose3d parentPose = poses.get(parent.name());
        if (parentPose != null) {
            Pose3d newPose = parentPose.transformBy(t);
            poses.put(jointName, newPose);
            return newPose;
        }
        Pose3d newParentPose = forward(robot, poses, parent.name(), q);
        Pose3d newNewPose = newParentPose.transformBy(t);
        poses.put(jointName, newNewPose);
        return newNewPose;
    }

    /** parent joint of the specified joint */
    public static URDFModel.Joint parentJoint(URDFModel.Robot robot, String childName) {
        URDFModel.Joint childJoint = getJoint(robot, childName);
        URDFModel.Link parentLink = childJoint.parent();
        for (URDFModel.Joint joint : robot.joints()) {
            if (joint.child() == parentLink)
                return joint;
        }
        return null;
    }

    /** Transform for a single joint */
    public static Transform3d jointTransform(URDFModel.Joint joint, double q) {
        return switch (joint.type()) {
            case revolute, continuous ->
                new Transform3d(0, 0, 0, new Rotation3d(joint.axis(), q))
                        .plus(new Transform3d(Pose3d.kZero, joint.origin()));
            case prismatic ->
                new Transform3d(new Translation3d(joint.axis().times(q)), Rotation3d.kZero)
                        .plus(new Transform3d(Pose3d.kZero, joint.origin()));
            case fixed ->
                new Transform3d(Pose3d.kZero, joint.origin());
            default ->
                throw new UnsupportedOperationException();
        };

    }

    public static URDFModel.Joint getJoint(URDFModel.Robot robot, String name) {
        for (URDFModel.Joint joint : robot.joints()) {
            if (joint.name().equals(name))
                return joint;
        }
        return null;
    }
}
