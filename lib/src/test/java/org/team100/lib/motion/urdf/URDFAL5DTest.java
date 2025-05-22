package org.team100.lib.motion.urdf;

import org.junit.jupiter.api.Test;

public class URDFAL5DTest {
    @Test
    void test1() {
        // to compute the end-effector forward kinematics for the URDF model, we need to
        // find the chain of joints from the root to the end. in general, URDF is a
        // tree, and so we could solve multiple ends simultaneously, but we never need
        // that; we only use it as a single open chain, from the root to the end, mapped
        // to the q vector.
        //
        // urdfpy FK uses a map from joint names to configurations, instead of a q
        // vector.
        //
        // ikpy uses "chains" instead of urdf
        //
        // pinocchio has its own model that it populates from urdf.
        //
        // our FK calculation wants Transform3d instances, one per joint (note that the
        // "joint" includes the link length)
    }

}
