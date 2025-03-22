package org.team100.lib.localization;

import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;

import org.team100.lib.config.Camera;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.EnumLogger;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.ValueEventData;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Extracts robot pose estimates from camera input.
 * 
 * This "24" version uses the "struct" method instead of the "msgpack" method,
 * which matches the TagFinder24 code on the camera.
 */
public class VisionDataProvider24 implements Glassy {
    private static final boolean DEBUG = false;
    /**
     * If the tag is closer than this threshold, then the camera's estimate of tag
     * rotation might be more accurate than the gyro, so we use the camera's
     * estimate of tag rotation to update the robot pose. If the tag is further away
     * than this, then the camera-derived rotation is probably less accurate than
     * the gyro, so we use the gyro instead.
     * 
     * Set this to zero to disable tag-derived rotation and always use the gyro.
     * 
     * Set this to some large number (e.g. 100) to disable gyro-derived rotation and
     * always use the camera.
     */
    private static final double kTagRotationBeliefThresholdMeters = 0;
    /** Discard results further than this from the previous one. */
    private static final double kVisionChangeToleranceMeters = 0.1;
    // private static final double kVisionChangeToleranceMeters = 1;

    /** this is the default value which, in hindsight, seems ridiculously high. */
    private static final double[] defaultStateStdDevs = new double[] {
            0.1,
            0.1,
            0.1 };
    /**
     * This value is tuned so that errors scale at 0.2x per second. See
     * SwerveDrivePoseEstimator100Test::testFirmerNudge.
     */
    static final double[] tightStateStdDevs = new double[] {
            0.001,
            0.001,
            0.1 };

    private final PoseEstimator100 m_poseEstimator;
    private final AprilTagFieldLayoutWithCorrectOrientation m_layout;
    private final PoseEstimationHelper m_helper;
    private final NetworkTableListenerPoller m_poller;
    private final StructArrayPublisher<Pose3d> m_pub_tags;

    // LOGGERS
    private final EnumLogger m_log_alliance;
    private final DoubleLogger m_log_heedRadius;

    // Remember the previous vision-based pose estimate, so we can measure the
    // distance
    // between consecutive updates, and ignore too-far updates.
    private Pose2d m_lastRobotInFieldCoords;

    // reuse the buffer since it takes some time to make
    private StructBuffer<Blip24> m_buf = StructBuffer.create(Blip24.struct);

    private double m_latestTimeSec = 0;

    /** use tags closer than this; ignore tags further than this. */
    private double m_heedRadiusM = 3.5;

    /**
     * @param layout
     * @param poseEstimator
     * @param rotationSupplier rotation for the given time in seconds
     * @throws IOException
     */
    public VisionDataProvider24(
            LoggerFactory parent,
            AprilTagFieldLayoutWithCorrectOrientation layout,
            PoseEstimator100 poseEstimator) {
        LoggerFactory child = parent.child(this);
        m_layout = layout;
        m_helper = new PoseEstimationHelper(child);
        m_poseEstimator = poseEstimator;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_poller = new NetworkTableListenerPoller(inst);
        m_poller.addListener(
                new MultiSubscriber(inst, new String[] { "vision" }),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll));
        m_pub_tags = inst.getStructArrayTopic("tags", Pose3d.struct).publish();
        m_log_alliance = child.enumLogger(Level.TRACE, "alliance");
        m_log_heedRadius = child.doubleLogger(Level.TRACE, "heed radius");
    }

    /**
     * The age of the last pose estimate, in microseconds.
     * The caller could use this to, say, indicate tag visibility.
     */
    public double getPoseAgeSec() {
        double now = Takt.get();
        return now - m_latestTimeSec;
    }

    /**
     * Read queued network input, and give it to the pose estimator.
     * 
     * This runs once per cycle, in SwerveDriveSubsystem.update() which is called by
     * Memo.updateAll(), which runs in Robot.robotPeriodic().
     */
    public void update() {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.isPresent()) {
            Util.warn("VisionDataProvider24: Alliance is not present!");
            return;
        }
        List<Pose3d> tags = new ArrayList<>();
        NetworkTableEvent[] events = m_poller.readQueue();
        for (NetworkTableEvent e : events) {
            ValueEventData ve = e.valueData;
            NetworkTableValue v = ve.value;
            String name = ve.getTopic().getName();
            String[] fields = name.split("/");
            if (fields.length != 4) {
                Util.warnf("VisionDataProvider24: weird event name: %s\n", name);
                continue;
            }
            if (fields[2].equals("fps")) {
                // FPS is not used by the robot
            } else if (fields[2].equals("latency")) {
                // latency is not used by the robot
            } else if (fields[3].equals("blips")) {
                // decode the way StructArrayEntryImpl does
                byte[] b = v.getRaw();
                if (b.length == 0) {
                    Util.warnf("VisionDataProvider24: no raw value for name: %s\n", name);
                    continue;
                }
                Blip24[] blips;
                try {
                    synchronized (m_buf) {
                        blips = m_buf.readArray(b);
                    }
                } catch (RuntimeException ex) {
                    Util.warnf("VisionDataProvider24: blip decoding failed for name: %s\n", name);
                    continue;
                }
                // the ID of the camera
                String cameraSerialNumber = fields[1];

                // Vasili added this extra delay after some experimentation that he should
                // describe here.
                final double IMPORTANT_MAGIC_NUMBER = 0.027;
                double blipTimeSec = (v.getServerTime() / 1000000.0 - IMPORTANT_MAGIC_NUMBER);
                List<Pose3d> newTags = estimateRobotPose(
                        cameraSerialNumber,
                        blips,
                        blipTimeSec,
                        alliance.get());
                tags.addAll(newTags);
            } else {
                // this event is not for us
                // Util.println("weird vision update key: " + name);
            }
        }
        // publish all the tags we've seen
        m_pub_tags.set(tags.toArray(new Pose3d[0]));
    }

    /**
     * Tags outside this radius are ignored.
     */
    public void setHeedRadiusM(double heedRadiusM) {
        m_heedRadiusM = heedRadiusM;
    }

    /**
     * @param estimateConsumer   is the pose estimator but exposing it here makes it
     *                           easier to test.
     * @param cameraSerialNumber the camera identity, obtained from proc/cpuinfo
     * @param blips              all the targets the camera sees right now
     * @return the apparent location of tags we can see
     */
    List<Pose3d> estimateRobotPose(
            String cameraSerialNumber,
            final Blip24[] blips,
            double blipTimeSec,
            Alliance alliance) {
        m_log_alliance.log(() -> alliance);
        final Transform3d cameraInRobotCoordinates = Camera.get(cameraSerialNumber).getOffset();

        final Rotation2d gyroRotation = m_poseEstimator.get(blipTimeSec).pose().getRotation();

        // double endTime = Takt.actual();
        // Util.printf("Time: %.4f\n", (endTime - startTime));

        return estimateFromBlips(
                blips,
                cameraInRobotCoordinates,
                blipTimeSec,
                gyroRotation,
                alliance);
    }

    private List<Pose3d> estimateFromBlips(
            final Blip24[] blips,
            final Transform3d cameraInRobotCoordinates,
            final double frameTimeSec,
            final Rotation2d gyroRotation,
            Alliance alliance) {
        m_log_heedRadius.log(() -> m_heedRadiusM);

        List<Pose3d> tags = new ArrayList<>();

        for (int i = 0; i < blips.length; ++i) {
            Blip24 blip = blips[i];

            if (DEBUG)
                Util.printf("blip %s\n", blip);

            // Skip too-far tags.
            if (blip.blipToTransform().getTranslation().getNorm() > m_heedRadiusM) {
                continue;
            }

            // Look up the pose of the tag in the field frame.
            Optional<Pose3d> tagInFieldCoordsOptional = m_layout.getTagPose(alliance, blip.getId());
            if (!tagInFieldCoordsOptional.isPresent()) {
                Util.warnf("VisionDataProvider24: no tag for id %d\n", blip.getId());
                continue;
            }
            final Pose3d tagInField = tagInFieldCoordsOptional.get();

            // Gyro only produces yaw so use zero roll and zero pitch
            final Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(
                    0, 0, gyroRotation.getRadians());

            // Tag as it appears in the camera frame.
            final Transform3d tagInCamera = blip.blipToTransform();

            // Robot in the field frame.
            final Pose3d robotPoseInField = m_helper.getRobotPoseInFieldCoords(
                    cameraInRobotCoordinates,
                    tagInField,
                    tagInCamera,
                    robotRotationInFieldCoordsFromGyro,
                    kTagRotationBeliefThresholdMeters);

            // Robot in field frame. We always use the gyro for rotation.
            final Pose2d currentRobotinFieldCoords = new Pose2d(
                    robotPoseInField.getTranslation().toTranslation2d(),
                    gyroRotation);

            if (!Experiments.instance.enabled(Experiment.HeedVision)) {
                // If we've turned vision off altogether, then don't apply this update to the
                // pose estimator.
                continue;
            }

            if (m_lastRobotInFieldCoords == null) {
                // Ignore the very first update since we have no idea if it is far from the
                // previous one.
                m_lastRobotInFieldCoords = currentRobotinFieldCoords;
                continue;
            }

            final double distanceM = distance(m_lastRobotInFieldCoords, currentRobotinFieldCoords);
            if (distanceM > kVisionChangeToleranceMeters) {
                // The new estimate is too far from the previous one: it's probably garbage.
                m_lastRobotInFieldCoords = currentRobotinFieldCoords;
                continue;
            }

            m_poseEstimator.put(
                    frameTimeSec,
                    currentRobotinFieldCoords,
                    stateStdDevs(),
                    visionMeasurementStdDevs(distanceM));

            m_latestTimeSec = Takt.get();
            m_lastRobotInFieldCoords = currentRobotinFieldCoords;
        }
        return tags;
    }

    static double[] stateStdDevs() {
        if (Experiments.instance.enabled(Experiment.AvoidVisionJitter)) {
            return tightStateStdDevs;
        }
        return defaultStateStdDevs;
    }

    /** This is an educated guess. */
    static double[] visionMeasurementStdDevs(double distanceM) {
        if (Experiments.instance.enabled(Experiment.AvoidVisionJitter)) {
            /*
             * NEW (3/12/25), 2 cm std dev seems kinda realistic for 1 m.
             * 
             * If it still jitters, try 0.03 or 0.05, but watch out for slow convergence.
             * 
             * TODO: gather some actual data.
             */
            final double k = 0.03;
            return new double[] {
                    (k * distanceM) + 0.01,
                    (k * distanceM) + 0.01,
                    Double.MAX_VALUE };
        }
        /*
         * Standard deviation of pose estimate, as a fraction of target range.
         * This is a guess based on figure 5 in the Apriltag2 paper:
         * https://april.eecs.umich.edu/media/media/pdfs/wang2016iros.pdf
         */
        final double k = 0.03;
        return new double[] {
                k * distanceM,
                k * distanceM,
                Double.MAX_VALUE };
    }

    ///////////////////////////////////////

    /** Distance between pose translations. */
    private static double distance(Pose2d a, Pose2d b) {
        // the translation distance is a little quicker to calculate and we don't care
        // about the "twist" curve measurement
        return GeometryUtil.distance(
                a.getTranslation(),
                b.getTranslation());
    }

}
