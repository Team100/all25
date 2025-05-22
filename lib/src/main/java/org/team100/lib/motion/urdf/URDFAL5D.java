package org.team100.lib.motion.urdf;

import java.util.List;

import org.team100.lib.motion.urdf.URDFModel.Joint;
import org.team100.lib.motion.urdf.URDFModel.Link;
import org.team100.lib.motion.urdf.URDFModel.Robot;
import org.team100.lib.motion.urdf.URDFModel.Joint.JointType;
import org.team100.lib.motion.urdf.URDFModel.Joint.Limit;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * An example of using the URDF model to describe the Lynxmotion arm.
 * 
 * The content came from here:
 * https://groups.google.com/g/moveit-users/c/g3vaTDQSRcQ
 */
public class URDFAL5D {
    public static final Robot ROBOT;
    static {
        Link base_link = new Link("base_link");
        Link base_pan_link = new Link("base_pan_link");
        Link upper_arm_link = new Link("upper_arm_link");
        Link lower_arm_link = new Link("lower_arm_link");
        Link link3 = new Link("link3");
        Link gripper = new Link("gripper");
        Link tool_center_point = new Link("tool_center_point");
        ROBOT = new Robot(
                "AL5D",
                List.of(
                        base_link,
                        base_pan_link,
                        upper_arm_link,
                        lower_arm_link,
                        link3,
                        gripper,
                        tool_center_point),
                List.of(
                        new Joint(
                                "base_pan",
                                JointType.revolute,
                                new Limit(1000, 0, 3.14, 0.5),
                                base_link,
                                base_pan_link,
                                new Pose3d(0, 0, 0.06731, new Rotation3d(0, 0, -1.57)),
                                VecBuilder.fill(0, 0, 1)),
                        new Joint(
                                "shoulder_tilt",
                                JointType.revolute,
                                new Limit(1000.0, 0.17, 2.97, 0.5),
                                base_pan_link,
                                upper_arm_link,
                                new Pose3d(),
                                VecBuilder.fill(0, -1, 0)),
                        new Joint(
                                "elbow_tilt",
                                JointType.revolute,
                                new Limit(1000.0, 0.52, 3.14, 0.5),
                                upper_arm_link,
                                lower_arm_link,
                                new Pose3d(0.14605, 0, 0, new Rotation3d(0, 3.14, 0)),
                                VecBuilder.fill(0, -1, 0)),
                        new Joint(
                                "wrist_tilt",
                                JointType.revolute,
                                new Limit(1000.0, 0.0, 3.14, 0.5),
                                lower_arm_link,
                                link3,
                                new Pose3d(0.187325, 0, 0, new Rotation3d(0, 1.57, 0)),
                                VecBuilder.fill(0, -1, 0)),
                        new Joint(
                                "wrist_rotate",
                                JointType.revolute,
                                new Limit(1000.0, 0.0, 3.14, 0.5),
                                link3,
                                gripper,
                                new Pose3d(0.034, 0, 0, new Rotation3d(-1.57, 0, 0)),
                                VecBuilder.fill(1, 0, 0)),
                        new Joint(
                                "center_point",
                                JointType.fixed, null,
                                gripper,
                                tool_center_point,
                                new Pose3d(0.055, 0, 0, new Rotation3d()), null)));
    }
}