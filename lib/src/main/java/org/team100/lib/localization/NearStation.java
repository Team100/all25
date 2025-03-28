package org.team100.lib.localization;

import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;

public class NearStation {
    private final Supplier <Pose2d> m_drive;

    public NearStation(Supplier<Pose2d> m_drive) {
        this.m_drive = m_drive;
    }

    public boolean closeToStation() {
        Pose2d pose = m_drive.get();
        return Math.hypot(pose.getX(), pose.getY()) <= 3 ||
                Math.hypot(pose.getX(), 8 - pose.getY()) <= 3;
    }

}
