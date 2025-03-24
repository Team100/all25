package org.team100.lib.localization;

import org.team100.lib.geometry.GeometryUtil;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Mirrors tag_finder24.py Blip24. */
public class Blip24 {
    private final int id;
    private final Transform3d pose;

    public Blip24(int id, Transform3d pose) {
        this.id = id;
        this.pose = pose;
    }

    public static Blip24 fromXForward(int id, Transform3d pose) {
        return new Blip24(id, new Transform3d(
                GeometryUtil.xForwardToZForward(pose.getTranslation()),
                GeometryUtil.xForwardToZForward(pose.getRotation())));
    }

    /**
     * ID of the AprilTag.
     */
    public int getId() {
        return id;
    }

    /**
     * Raw transform produced by the camera. The camera's coordinate system has X to
     * the right, Y down, and Z forward, so this Transform3d should not be used
     * directly.
     */
    public Transform3d getRawPose() {
        return pose;
    }

    /**
     * Extract translation and rotation from z-forward blip and return the same
     * translation and rotation as an NWU x-forward transform. Package-private for
     * testing.
     */
    public Transform3d blipToTransform() {
        return new Transform3d(blipToTranslation(), blipToRotation());
    }

    @Override
    public String toString() {
        return "Blip24 [id=" + id + ", pose=" + pose + "]";
    }

    public static final Blip24Struct struct = new Blip24Struct();

    /**
     * Extract the translation from a "z-forward" blip and return the same
     * translation expressed in our usual "x-forward" NWU translation.
     * It would be possible to also consume the blip rotation matrix, if it were
     * renormalized, but it's not very accurate, so we don't consume it.
     * Package-private for testing.
     */
    private Translation3d blipToTranslation() {
        return GeometryUtil.zForwardToXForward(pose.getTranslation());
    }

    /**
     * Extract the rotation from the "z forward" blip and return the same rotation
     * expressed in our usual "x forward" NWU coordinates. Package-private for
     * testing.
     */
    private Rotation3d blipToRotation() {
        return GeometryUtil.zForwardToXForward(pose.getRotation());
    }
}
