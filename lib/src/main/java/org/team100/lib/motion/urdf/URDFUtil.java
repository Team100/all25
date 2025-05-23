package org.team100.lib.motion.urdf;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.math.NewtonsMethod;
import org.team100.lib.motion.urdf.URDFModel.Joint.JointType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N6;

/** Maybe some of this should go in the model itself. */
public class URDFUtil {

    /** All joint configurations. */
    public static Map<String, Double> inverse(
            URDFModel.Robot robot,
            String jointName,
            Pose3d x) {
        // there are 6 joints but one is "fixed"
        Nat<N5> configDim = Nat.N5();
        Nat<N6> twistDim = Nat.N6();
        Function<Vector<N5>, Vector<N6>> f = q -> {
            System.out.println(q);
            // solve all of them
            Map<String, Double> qMap = qMap(robot, q);
            Map<String, Pose3d> p = forward(robot, qMap);
            // pick out the one we want
            Pose3d pose = p.get(jointName);
            Vector<N6> pv = GeometryUtil.toVec(GeometryUtil.slog(pose));
            System.out.printf("pose %s pv %s\n", pose, pv);
            return pv;
        };
        NewtonsMethod<N5, N6> solver = new NewtonsMethod<>(configDim, twistDim, f, 1e-3, 10);
        Vector<N5> q0 = VecBuilder.fill(0.1, 0.1, 0.1, 0.1, 0.1);
        Vector<N6> Xd = GeometryUtil.toVec(GeometryUtil.slog(x));
        Vector<N5> q = solver.solve2(q0, Xd);
        return qMap(robot, q);
    }

    static <Q extends Num> Map<String, Double> qMap(URDFModel.Robot robot, Vector<Q> q) {
        Map<String, Double> qMap = new HashMap<>();
        List<URDFModel.Joint> joints = robot.joints();
        for (int i = 0; i < joints.size(); ++i) {
            URDFModel.Joint joint = joints.get(i);
            if (joint.type() == JointType.revolute
                    || joint.type() == JointType.continuous
                    || joint.type() == JointType.prismatic) {
                // skip fixed since it doesn't have a parameter and thus breaks the solver
                qMap.put(joint.name(), q.get(i));
            }
        }
        return qMap;
    }

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

    /**
     * Transform for a single joint.
     * q can be null, for fixed joints.
     */
    public static Transform3d jointTransform(URDFModel.Joint joint, Double q) {
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
