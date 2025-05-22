package org.team100.lib.motion.urdf;

import org.junit.jupiter.api.Test;

public class URDFUtilTest {
    @Test
    void test1() {
        URDFModel.Robot m = URDFAL5D.ROBOT;
        URDFModel.Joint j = URDFUtil.getJoint(m, "center_point");
        
    }
}
