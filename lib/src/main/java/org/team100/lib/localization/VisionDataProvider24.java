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
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.EnumLogger;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
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
     * Five cameras, 50hz each => 250 hz of updates. Rio runs at 50 hz, so there
     * should be five messages waiting for us each cycle.
     */
    private static final int QUEUE_DEPTH = 10;
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
    private final NetworkTableListenerPoller m_poller;
    /**
     * The apparent position of tags we see: this can be shown in AdvantageScope
     * using the Vision Target feature. The apparent position should match the
     * actual position, if the cameras are calibrated correctly. Note this involves
     * matching the frame timestamp with the pose history timestamp, so if the blip
     * source timestamp is wrong (as it is at the moment in the simulated tag
     * detector) then these positions will be a little bit wrong.
     */
    private final StructArrayPublisher<Pose3d> m_pub_tags;
    /** Just tags we use for pose estimation. */
    private final StructArrayPublisher<Pose3d> m_pub_used_tags;

    /**
     * The pose we derive from each sighting, so we can see it in AdvantageScope's
     * map, which can't understand our usual Pose2dLogger's output.
     */
    private final StructPublisher<Pose2d> m_pub_pose;

    // LOGGERS
    private final EnumLogger m_log_alliance;
    private final DoubleLogger m_log_heedRadius;
    private final BooleanLogger m_log_using_gyro;
    private final DoubleLogger m_log_tag_error;
    private final Pose2dLogger m_log_pose;
    /**
     * The difference between the current instant and the instant of the blip,
     * including our magic correction, i.e. this is the time we look up in the pose
     * buffer.
     */
    private final DoubleLogger m_log_lag;

    // Remember the previous vision-based pose estimate, so we can measure the
    // distance
    // between consecutive updates, and ignore too-far updates.
    private Pose2d m_prevPose;

    // reuse the buffer since it takes some time to make
    private StructBuffer<Blip24> m_buf = StructBuffer.create(Blip24.struct);

    private double m_latestTimeSec = 0;

    /** use tags closer than this; ignore tags further than this. */
    private double m_heedRadiusM = 3.5;

    /**
     * Accumulates all tags we receive in each cycle, whether we use them or not.
     */
    private final List<Pose3d> m_allTags = new ArrayList<>();
    /**
     * Just the tags we use for pose estimation, i.e. not ones that are too far
     * away.
     */
    private final List<Pose3d> m_usedTags = new ArrayList<>();

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
        m_poseEstimator = poseEstimator;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_poller = new NetworkTableListenerPoller(inst);
        m_poller.addListener(
                new MultiSubscriber(
                        inst,
                        new String[] { "vision" },
                        PubSubOption.pollStorage(QUEUE_DEPTH)),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll));
        m_pub_tags = inst.getStructArrayTopic("tags", Pose3d.struct).publish();
        m_pub_used_tags = inst.getStructArrayTopic("used tags", Pose3d.struct).publish();

        m_pub_pose = inst.getStructTopic("pose", Pose2d.struct).publish();

        m_log_alliance = child.enumLogger(Level.TRACE, "alliance");
        m_log_heedRadius = child.doubleLogger(Level.TRACE, "heed radius");
        m_log_using_gyro = child.booleanLogger(Level.TRACE, "rotation source");
        m_log_tag_error = child.doubleLogger(Level.TRACE, "tag error");
        m_log_pose = child.pose2dLogger(Level.TRACE, "pose");
        m_log_lag = child.doubleLogger(Level.TRACE, "lag");
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
            // this happens on startup
            // Util.warn("VisionDataProvider24: Alliance is not present!");
            return;
        }
        m_log_alliance.log(() -> alliance.get());

        // only publish new sights
        m_allTags.clear();
        m_usedTags.clear();
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
                    // this seems to happen not-never which is weird.
                    // TODO: why?
                    // Util.warnf("VisionDataProvider24: no raw value for name: %s\n", name);
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
                String cameraId = fields[1];

                // Vasili added this extra delay after some experimentation that he should
                // describe here.
                final double IMPORTANT_MAGIC_NUMBER = 0.027;
                // server time is in microseconds
                // https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-intro.html#timestamps
                long serverTimeUs = v.getServerTime();
                double blipTimeSec = (serverTimeUs / 1000000.0 - IMPORTANT_MAGIC_NUMBER);

                // this seems to always be 1. ????
                m_log_lag.log(() -> Takt.get() - blipTimeSec);

                estimateRobotPose(cameraId, blips, blipTimeSec, alliance.get());
            } else {
                // this event is not for us
                // Util.println("weird vision update key: " + name);
            }
        }
        // publish all the tags we've seen
        // TODO: publish blank if we haven't seen any for awhile
        // TODO: Network Tables ignores dupliciates, which makes it
        // work poorly in simulation, so fix that.
        if (m_allTags.size() > 0)
            m_pub_tags.set(m_allTags.toArray(new Pose3d[0]));
        if (m_usedTags.size() > 0)
            m_pub_used_tags.set(m_usedTags.toArray(new Pose3d[0]));
    }

    /**
     * Tags outside this radius are ignored.
     */
    public void setHeedRadiusM(double heedRadiusM) {
        m_heedRadiusM = heedRadiusM;
    }

    /**
     * Compute the robot pose and put it in the pose estimator.
     * 
     * @param cameraId  From proc/cpuinfo
     * @param blips     The targets in the current camera frame
     * @param timestamp Camera frame timestamp
     * @param alliance  From the driver station
     * @return The apparent location of tags we can see
     */
    void estimateRobotPose(String cameraId, Blip24[] blips, double timestamp, Alliance alliance) {
        m_log_heedRadius.log(() -> m_heedRadiusM);

        // Robot-to-camera, offset from Camera.java
        final Transform3d cameraInRobot = Camera.get(cameraId).getOffset();

        // The pose from the frame timestamp.
        final Pose2d historicalPose = m_poseEstimator.get(timestamp).pose();

        // Field-to-robot
        Pose3d historicalPose3d = new Pose3d(historicalPose);
        // Field-to-robot plus robot-to-camera = field-to-camera
        Pose3d historicalCameraInField = historicalPose3d.transformBy(cameraInRobot);

        // The gyro rotation for the frame timestamp
        final Rotation2d gyroRotation = historicalPose.getRotation();

        for (int i = 0; i < blips.length; ++i) {
            Blip24 blip = blips[i];

            // if (DEBUG) {
            //     Translation3d t = blip.getRawPose().getTranslation();
            //     Rotation3d r = blip.getRawPose().getRotation();
            //     Util.printf("blip raw pose  %d X %5.2f Y %5.2f Z %5.2f R %5.2f P %5.2f Y %5.2f\n",
            //             blip.getId(), t.getX(), t.getY(), t.getZ(), r.getX(), r.getY(), r.getZ());
            // }

            // Look up the pose of the tag in the field frame.
            Optional<Pose3d> tagInFieldCoordsOptional = m_layout.getTagPose(alliance, blip.getId());
            if (!tagInFieldCoordsOptional.isPresent()) {
                // This shouldn't happen, but it does.
                Util.warnf("VisionDataProvider24: no tag for id %d\n", blip.getId());
                continue;
            }

            // Field-to-tag, canonical pose from the JSON file
            final Pose3d tagInField = tagInFieldCoordsOptional.get();

            // Camera-to-tag, as it appears in the camera frame.
            Transform3d tagInCamera = blip.blipToTransform();

            if (DEBUG) {
                // This is used for camera offset calibration. Place a tag at a known position,
                // observe the offset, and add it to Camera.java, inverted.
                Transform3d tagInRobot = cameraInRobot.plus(tagInCamera);
                Util.printf("tagInRobot id %d X %5.2f Y %5.2f Z %5.2f R %5.2f P %5.2f Y %5.2f\n",
                        blip.getId(),
                        tagInRobot.getTranslation().getX(),
                        tagInRobot.getTranslation().getY(),
                        tagInRobot.getTranslation().getZ(),
                        tagInRobot.getRotation().getX(),
                        tagInRobot.getRotation().getY(),
                        tagInRobot.getRotation().getZ());
            }

            if (tagInCamera.getTranslation().getNorm() > kTagRotationBeliefThresholdMeters) {
                // If the tag is further than the threshold, replace the tag rotation with
                // a rotation derived from the gyro.
                m_log_using_gyro.log(() -> true);
                tagInCamera = PoseEstimationHelper.tagInCamera(
                        cameraInRobot,
                        tagInField,
                        tagInCamera,
                        new Rotation3d(gyroRotation));
            } else {
                m_log_using_gyro.log(() -> false);
            }

            // given the historical pose, where do we think the tag is?
            Pose3d estimatedTagInField = historicalCameraInField.transformBy(tagInCamera);
            m_allTags.add(estimatedTagInField);

            // log the norm of the translational error of the tag.
            Transform3d tagError = tagInField.minus(estimatedTagInField);
            m_log_tag_error.log(() -> tagError.getTranslation().getNorm());

            // Estimate of robot pose.
            Pose3d pose3d = PoseEstimationHelper.robotInField(
                    cameraInRobot,
                    tagInField,
                    tagInCamera);

            // Robot in field frame. We always use the gyro for rotation.
            final Pose2d pose = new Pose2d(
                    pose3d.getTranslation().toTranslation2d(),
                    gyroRotation);

            m_log_pose.log(() -> pose);
            m_pub_pose.set(pose);

            if (!Experiments.instance.enabled(Experiment.HeedVision)) {
                // If we've turned vision off altogether, then don't apply this update to the
                // pose estimator.
                continue;
            }

            if (blip.blipToTransform().getTranslation().getNorm() > m_heedRadiusM) {
                // Skip too-far tags.
                continue;
            }

            if (m_prevPose == null) {
                // Ignore the very first update since we have no idea if it is far from the
                // previous one.
                m_prevPose = pose;
                continue;
            }

            final double distanceM = distance(m_prevPose, pose);
            if (distanceM > kVisionChangeToleranceMeters) {
                // The new estimate is too far from the previous one: it's probably garbage.
                m_prevPose = pose;
                continue;
            }

            m_usedTags.add(estimatedTagInField);
            m_poseEstimator.put(
                    timestamp,
                    pose,
                    stateStdDevs(),
                    visionMeasurementStdDevs(distanceM));

            m_latestTimeSec = Takt.get();
            m_prevPose = pose;
        }
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
