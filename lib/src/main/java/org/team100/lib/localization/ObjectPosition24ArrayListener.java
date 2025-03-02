package org.team100.lib.localization;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;

import org.team100.lib.config.Camera;
import org.team100.lib.config.Identity;
import org.team100.lib.config.SimulatedCamera;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.MultiSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.ValueEventData;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Listen for updates from the object-detector camera and remember them for
 * awhile.
 */
public class ObjectPosition24ArrayListener {
    /** Ignore sights older than this. */
    private static final double kMaxSightAgeS = 0.1;
    private StructBuffer<Rotation3d> m_buf = StructBuffer.create(Rotation3d.struct);
    private List<Translation2d> objects = new ArrayList<>();
    private final PoseEstimator100 m_poseSupplier;
    private final NetworkTableListenerPoller m_poller;

    private double latestTime = 0;

    public ObjectPosition24ArrayListener(PoseEstimator100 poseEstimator) {
        m_poseSupplier = poseEstimator;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_poller = new NetworkTableListenerPoller(inst);
        m_poller.addListener(
                new MultiSubscriber(
                        inst,
                        new String[] { "objectVision" },
                        PubSubOption.keepDuplicates(true)),
                EnumSet.of(NetworkTableEvent.Kind.kValueAll));
    }

    public void update() {
        for (NetworkTableEvent e : m_poller.readQueue()) {
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
            } else if (fields[3].equals("Rotation3d")) {
                // decode the way StructArrayEntryImpl does
                byte[] b = v.getRaw();
                if (b.length == 0) {
                    return;
                }
                // object! sights are x-ahead WPI coordinates, not z-ahead camera coordinates.
                Rotation3d[] sights;
                try {
                    synchronized (m_buf) {
                        sights = m_buf.readArray(b);
                        latestTime = Takt.get();
                    }
                } catch (RuntimeException ex) {
                    return;
                }
                Transform3d cameraInRobotCoordinates = Camera.get(fields[1]).getOffset();
                Pose2d robotPose = m_poseSupplier.get(v.getServerTime() / 1000000.0).pose();
                objects = TargetLocalizer.cameraRotsToFieldRelativeArray(
                        robotPose,
                        cameraInRobotCoordinates,
                        sights);
            } else {
                Util.warn("object weird vision update key: " + name);
            }
        }
    }

    /**
     * Field-relative translations of recent sights.
     */
    public List<Translation2d> getTranslation2dArray() {
        update();
        switch (Identity.instance) {
            case BLANK:
                Pose2d robotPose = m_poseSupplier.get(Takt.get()).pose();
                SimulatedCamera simCamera = SimulatedCamera.getGamePieceCamera();
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isEmpty())
                    return new ArrayList<>();
                List<Rotation3d> rot = simCamera.getKnownLocations(alliance.get(), robotPose);
                return TargetLocalizer.cameraRotsToFieldRelativeArray(
                        robotPose,
                        simCamera.getOffset(),
                        rot.toArray(new Rotation3d[0]));
            default:
                if (latestTime > Takt.get() - kMaxSightAgeS) {
                    return objects;
                }
                return new ArrayList<>();
        }
    }

    /**
     * The field-relative translation of the closest object, if any.
     */
    public Optional<Translation2d> getClosestTranslation2d() {
        update();
        Pose2d robotPose = m_poseSupplier.get(Takt.get()).pose();
        return ObjectPicker.closestObject(
                getTranslation2dArray(),
                robotPose);
    }

    /**
     * The field-relative translation of the closest object, if any.
     */
    public Translation2d getClosestTranslation2dNull() {
        Optional<Translation2d> translation2d = getClosestTranslation2d(); 
        if (translation2d.isEmpty()) {
            return null;
        }
        return getClosestTranslation2d().get();
    }
}