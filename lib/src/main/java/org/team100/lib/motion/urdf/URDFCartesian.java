package org.team100.lib.motion.urdf;

import java.util.List;

import org.team100.lib.motion.urdf.URDFModel.Robot;

/**
 * Cartesian prismatic, like the Haas mill.
 */
public class URDFCartesian {
    public static final Robot ROBOT;
    static {
            ROBOT = new Robot(
                "Cartesian",
                List.of(), 
                List.of());
        }
}
