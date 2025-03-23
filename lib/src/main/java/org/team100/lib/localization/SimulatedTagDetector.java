package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleFunction;

import org.team100.lib.config.Camera;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Publishes AprilTag Blip24 sightings on Network Tables, just like real
 * cameras would.
 */
public class SimulatedTagDetector {
    private static final boolean DEBUG = false;
    private static final boolean PUBLISH_DEBUG = false;
    // these are the extents of the normalized image coordinates
    // i.e. in WPILib coordinates this would be Y/X and Z/X.
    // our real cameras can see horizontally to about
    // 0.8 on each side. we didn't measure the vertical
    // extent, but it's probably something like 0.6.
    // TODO: make these parameters, to accommodate different sensors and lenses.
    //
    // see
    // https://docs.google.com/spreadsheets/d/1x2_58wyVb5e9HJW8WgakgYcOXgPaJe0yTIHew206M-M
    private static final double kHFOV = 0.8;
    private static final double kVFOV = 0.6;
    private static final int kTagCount = 22;
    // past about 80 degrees, you can't see the tag.
    private static final double kObliqueLimitRad = 1.4;
    // camera frame is from 85 ms ago
    // TODO: make this jitter a little
    private static final double kDelayS = 0.085;

    private final List<Camera> m_cameras;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;
    private final DoubleFunction<Pose2d> m_robotPose;

    private final Map<Camera, StructArrayPublisher<Blip24>> m_publishers;
    private final NetworkTableInstance m_inst;

    /**
     * 
     * @param cameras
     * @param layout
     * @param robotPose look up pose by timestamp (sec)
     */
    public SimulatedTagDetector(
            List<Camera> cameras,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            DoubleFunction<Pose2d> robotPose) {
        m_cameras = cameras;
        m_layout = layout;
        m_robotPose = robotPose;
        m_publishers = new HashMap<>();
        m_inst = NetworkTableInstance.create();
        m_inst.startClient4("SimulatedTagDetector");
        m_inst.setServer("localhost");
        for (Camera camera : m_cameras) {
            // see tag_detector.py
            String path = "vision/" + camera.getSerial() + "/0";
            String name = path + "/blips";
            var topic = m_inst.getStructArrayTopic(name, Blip24.struct);
            StructArrayPublisher<Blip24> publisher = topic.publish();
            m_publishers.put(camera, publisher);
        }
    }

    public void periodic() {
        if (DEBUG)
            Util.println("simulated tag detector");
        Optional<Alliance> opt = DriverStation.getAlliance();
        if (opt.isEmpty())
            return;
        double timestampS = Takt.get() - kDelayS;
        Pose3d robotPose3d = new Pose3d(m_robotPose.apply(timestampS));
        if (DEBUG) {
            Util.printf("robot pose X %6.2f Y %6.2f Z %6.2f R %6.2f P %6.2f Y %6.2f \n",
                    robotPose3d.getTranslation().getX(),
                    robotPose3d.getTranslation().getY(),
                    robotPose3d.getTranslation().getZ(),
                    robotPose3d.getRotation().getX(),
                    robotPose3d.getRotation().getY(),
                    robotPose3d.getRotation().getZ());
        }
        for (Camera camera : m_cameras) {
            List<Blip24> blips = new ArrayList<>();
            Transform3d cameraOffset = camera.getOffset();
            Pose3d cameraPose3d = robotPose3d.plus(cameraOffset);
            Alliance alliance = opt.get();

            for (int tagId = 1; tagId <= kTagCount; ++tagId) {
                if (DEBUG) {
                    Util.printf("alliance %s camera %12s ", alliance.name(), camera.name());
                }
                Pose3d tagPose = m_layout.getTagPose(alliance, tagId).get();
                if (DEBUG) {
                    Util.printf("tag id: %2d tag pose: X %6.2f Y %6.2f Z %6.2f R %6.2f P %6.2f Y %6.2f ",
                            tagId,
                            tagPose.getTranslation().getX(),
                            tagPose.getTranslation().getY(),
                            tagPose.getTranslation().getZ(),
                            tagPose.getRotation().getX(),
                            tagPose.getRotation().getY(),
                            tagPose.getRotation().getZ());
                }
                Transform3d tagInCamera = tagInCamera(cameraPose3d, tagPose);
                if (visible(tagInCamera)) {
                    // publish it
                    if (DEBUG) {
                        Util.printf("VISIBLE ");
                    }
                    blips.add(Blip24.fromXForward(tagId, tagInCamera));
                } else {
                    // ignore it
                    if (DEBUG) {
                        Util.printf(" . ");
                    }
                }
                if (DEBUG) {
                    Util.printf("camera: X %6.2f Y %6.2f Z %6.2f R %6.2f P %6.2f Y %6.2f",
                            cameraOffset.getTranslation().getX(),
                            cameraOffset.getTranslation().getY(),
                            cameraOffset.getTranslation().getZ(),
                            cameraOffset.getRotation().getX(),
                            cameraOffset.getRotation().getY(),
                            cameraOffset.getRotation().getZ());
                    Translation3d tagTranslationInCamera = tagInCamera.getTranslation();
                    Rotation3d tagRotationInCamera = tagInCamera.getRotation();

                    Util.printf(" tag in camera: X %6.2f Y %6.2f Z %6.2f  R %6.2f P %6.2f Y %6.2f\n",
                            tagTranslationInCamera.getX(),
                            tagTranslationInCamera.getY(),
                            tagTranslationInCamera.getZ(),
                            tagRotationInCamera.getX(),
                            tagRotationInCamera.getY(),
                            tagRotationInCamera.getZ());
                }

            }

            // publish whatever we saw
            // TODO: pose should be from the past, using publisher "set" from the past.
            // use a microsecond timestamp as specified here
            // https://docs.wpilib.org/en/stable/docs/software/networktables/publish-and-subscribe.html

            // guess about a reasonable delay
            long delayUs = (long) kDelayS * 1000000;
            long timestampUs = NetworkTablesJNI.now();
            m_publishers.get(camera).set(blips.toArray(new Blip24[0]), timestampUs - delayUs);
            if (PUBLISH_DEBUG) {
                Util.printf("%s\n", blips);
            }
        }

    }

    /** Return the transform from the camera pose to the tag pose. */
    static Transform3d tagInCamera(Pose3d cameraPose3d, Pose3d tagPose) {
        return new Transform3d(cameraPose3d, tagPose);
    }

    /**
     * If the target is behind the camera, it is never visible.
     */
    static boolean inFront(Transform3d tagInCamera) {
        Translation3d tagTranslationInCamera = tagInCamera.getTranslation();
        double x = tagTranslationInCamera.getX();
        if (x < 0) {
            if (DEBUG)
                Util.printf("   behind (%6.2f) ", x);
            return false;
        }
        if (DEBUG)
            Util.printf(" in front (%6.2f) ", x);
        return true;
    }

    /**
     * The tag needs to be facing the camera, at least a little.
     * 
     * We compute the angle between the tag normal vector and the translation
     * vector to find the apparent angle.
     */
    static boolean facing(Transform3d tagInCamera) {
        Translation3d tagTranslationInCamera = tagInCamera.getTranslation();
        Rotation3d tagRotationInCamera = tagInCamera.getRotation();
        Translation3d normal = new Translation3d(1, 0, 0);
        // this points "into the page" of the tag
        Translation3d rotatedNormal = normal.rotateBy(tagRotationInCamera);
        Vector<N3> rotatedNormalVector = rotatedNormal.toVector();
        Vector<N3> tagTranslationVector = tagTranslationInCamera.toVector();
        Rotation3d apparentAngle = new Rotation3d(tagTranslationVector, rotatedNormalVector);
        double angle = apparentAngle.getAngle();

        if (Math.abs(angle) > kObliqueLimitRad) {
            if (DEBUG)
                Util.printf(" facing away (%6.2f)", angle);
            return false;
        }
        if (DEBUG)
            Util.printf("    angle ok (%6.2f)", angle);
        return true;
    }

    /**
     * The "field of view" is expressed as an angle, but we don't really use an
     * angle, we use the pinhole projection.
     * opencv notation for these normalized coordinates is
     * x'' and y'' so these are x-prime-prime.
     * x is the horizontal dimension, pointing right
     * y is the vertical dimension, pointing down
     * the origin is on the camera bore.
     */
    static boolean inFOV(Transform3d tagInCamera) {
        Translation3d tagTranslationInCamera = tagInCamera.getTranslation();
        double xpp = -1.0 * tagTranslationInCamera.getY() / tagTranslationInCamera.getX();
        double ypp = -1.0 * tagTranslationInCamera.getZ() / tagTranslationInCamera.getX();
        if (Math.abs(xpp) < kHFOV && Math.abs(ypp) < kVFOV) {
            if (DEBUG)
                Util.printf("  FOV IN xpp %6.2f ypp %6.2f ", xpp, ypp);
            return true;
        }
        if (DEBUG)
            Util.printf(" FOV OUT xpp %6.2f ypp %6.2f ", xpp, ypp);
        return false;

    }

    static boolean visible(Transform3d tagInCamera) {
        if (!inFront(tagInCamera)) {
            if (DEBUG)
                Util.printf(" ........................................................");
            return false;
        }

        if (!facing(tagInCamera)) {
            if (DEBUG)
                Util.printf(" ...................................");
            return false;
        }

        if (!inFOV(tagInCamera)) {
            if (DEBUG)
                Util.printf(" ... ");
            return false;
        }

        return true;
    }

}
