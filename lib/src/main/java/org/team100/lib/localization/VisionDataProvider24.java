package org.team100.lib.localization;

import java.io.IOException;
import java.util.EnumSet;
import java.util.Optional;

import org.team100.lib.config.Camera;
import org.team100.lib.dashboard.Glassy;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.EnumLogger;
import org.team100.lib.util.Takt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.networktables.NetworkTableValue;
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
public class VisionDataProvider24 implements VisionData, Glassy {
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
    // LOGGERS
    private final EnumLogger m_log_alliance;

    // for blip filtering
    private Pose2d lastRobotInFieldCoords;

    // reuse the buffer since it takes some time to make
    private StructBuffer<Blip24> m_buf = StructBuffer.create(Blip24.struct);

    private double latestTimeSec = 0;

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
        m_log_alliance = child.enumLogger(Level.TRACE, "alliance");
    }

    /**
     * The age of the last pose estimate, in microseconds.
     * The caller could use this to, say, indicate tag visibility.
     */
    public double getPoseAgeSec() {
        double now = Takt.get();
        return now - latestTimeSec;
    }

    public void update() {
        NetworkTableEvent[] events = m_poller.readQueue();
        for (NetworkTableEvent e : events) {
            ValueEventData ve = e.valueData;
            NetworkTableValue v = ve.value;
            String name = ve.getTopic().getName();
            String[] fields = name.split("/");
            if (fields.length != 4) {
                return;
            }
            if (fields[2].equals("fps")) {
                // FPS is not used by the robot
            } else if (fields[2].equals("latency")) {
                // latency is not used by the robot
            } else if (fields[3].equals("blips")) {
                // decode the way StructArrayEntryImpl does
                byte[] b = v.getRaw();
                if (b.length == 0)
                    return;
                Blip24[] blips;
                try {
                    synchronized (m_buf) {
                        blips = m_buf.readArray(b);
                    }
                } catch (RuntimeException ex) {
                    return;
                }
                // the ID of the camera
                String cameraSerialNumber = fields[1];

                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (!alliance.isPresent())
                    return;

                // Vasili added this extra delay after some experimentation that he should
                // describe here.
                final double IMPORTANT_MAGIC_NUMBER = 0.027;
                double blipTimeSec = (v.getServerTime() / 1000000.0 - IMPORTANT_MAGIC_NUMBER);
                estimateRobotPose(
                        cameraSerialNumber,
                        blips,
                        blipTimeSec,
                        alliance.get());
            } else {
                // this event is not for us
                // Util.println("weird vision update key: " + name);
            }
        }
    }

    /**
     * @param estimateConsumer   is the pose estimator but exposing it here makes it
     *                           easier to test.
     * @param cameraSerialNumber the camera identity, obtained from proc/cpuinfo
     * @param blips              all the targets the camera sees right now
     */
    void estimateRobotPose(
            String cameraSerialNumber,
            final Blip24[] blips,
            double blipTimeSec,
            Alliance alliance) {

        m_log_alliance.log(() -> alliance);
        final Transform3d cameraInRobotCoordinates = Camera.get(cameraSerialNumber).getOffset();

        final Rotation2d gyroRotation = m_poseEstimator.get(blipTimeSec).pose().getRotation();

        // double endTime = Takt.actual();
        // Util.printf("Time: %.4f\n", (endTime - startTime));

        estimateFromBlips(
                blips,
                cameraInRobotCoordinates,
                blipTimeSec,
                gyroRotation,
                alliance);

    }

    private void estimateFromBlips(
            final Blip24[] blips,
            final Transform3d cameraInRobotCoordinates,
            final double frameTimeSec,
            final Rotation2d gyroRotation,
            Alliance alliance) {
        for (int i = 0; i < blips.length; ++i) {
            Blip24 blip = blips[i];

            Optional<Pose3d> tagInFieldCoordsOptional = m_layout.getTagPose(alliance, blip.getId());
            if (!tagInFieldCoordsOptional.isPresent())
                continue;

            int distanceTillIgnoreM = 5;
            if (blip.getPose().getTranslation().getNorm() > distanceTillIgnoreM) {
                return;
            }

            // Gyro only produces yaw so use zero roll and zero pitch
            Rotation3d robotRotationInFieldCoordsFromGyro = new Rotation3d(
                    0, 0, gyroRotation.getRadians());

            Pose3d tagInFieldCoords = tagInFieldCoordsOptional.get();
            Pose3d robotPoseInFieldCoords = m_helper.getRobotPoseInFieldCoords(
                    cameraInRobotCoordinates,
                    tagInFieldCoords,
                    blip,
                    robotRotationInFieldCoordsFromGyro,
                    kTagRotationBeliefThresholdMeters);

            Translation2d robotTranslationInFieldCoords = robotPoseInFieldCoords.getTranslation().toTranslation2d();

            Pose2d currentRobotinFieldCoords = new Pose2d(robotTranslationInFieldCoords, gyroRotation);

            if (!Experiments.instance.enabled(Experiment.HeedVision))
                continue;

            if (lastRobotInFieldCoords != null) {
                // the translation distance is a little quicker to calculate and we don't care
                // about the "twist" curve measurement
                double distanceM = GeometryUtil.distance(
                        lastRobotInFieldCoords.getTranslation(),
                        currentRobotinFieldCoords.getTranslation());
                if (distanceM <= kVisionChangeToleranceMeters) {
                    // this hard limit excludes false positives, which were a bigger problem in 2023
                    // due to the coarse tag family used. in 2024 this might not be an issue.
                    latestTimeSec = Takt.get();
                    m_poseEstimator.put(
                            frameTimeSec,
                            currentRobotinFieldCoords,
                            stateStdDevs(),
                            visionMeasurementStdDevs(distanceM));
                }
            }
            lastRobotInFieldCoords = currentRobotinFieldCoords;
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
            final double k = 0.02;
            return new double[] {
                    k * distanceM,
                    k * distanceM,
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

}
