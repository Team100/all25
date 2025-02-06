package org.team100.lib.follower;

/**
 * Known-good controller settings.
 */
public class DriveTrajectoryFollowerFactory {

    private final DriveTrajectoryFollowerUtil m_util;

    public DriveTrajectoryFollowerFactory(DriveTrajectoryFollowerUtil util) {
        m_util = util;
    }

    public FieldRelativeDriveTrajectoryFollower fieldRelativeFancyPIDF(FieldRelativeDrivePIDFFollower.Log log) {
        return new FieldRelativeDrivePIDFFollower(log, m_util, false, 2.4, 1.3);
    }

    public FieldRelativeDriveTrajectoryFollower fieldRelativeGoodPIDF(FieldRelativeDrivePIDFFollower.Log log) {
        return new FieldRelativeDrivePIDFFollower(log, m_util, false, 1, 1.3);
    }

    public FieldRelativeDriveTrajectoryFollower autoFieldRelativePIDF(FieldRelativeDrivePIDFFollower.Log log) {
        return new FieldRelativeDrivePIDFFollower(log, m_util, false, 1, 1.3);
    }

    public FieldRelativeDriveTrajectoryFollower fieldRelativeFfOnly(FieldRelativeDrivePIDFFollower.Log log) {
        return new FieldRelativeDrivePIDFFollower(log, m_util, true, 2.4, 1.3);
    }

    public FieldRelativeDriveTrajectoryFollower testFieldRelativePIDF(FieldRelativeDrivePIDFFollower.Log log) {
        return new FieldRelativeDrivePIDFFollower(log, m_util, false, 2.4, 2.4);
    }

    public FieldRelativeDriveTrajectoryFollower testFieldRelativeFFOnly(FieldRelativeDrivePIDFFollower.Log log) {
        return new FieldRelativeDrivePIDFFollower(log, m_util, true, 2.4, 2.4);
    }

}
