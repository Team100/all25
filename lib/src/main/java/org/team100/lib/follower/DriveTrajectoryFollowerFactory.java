package org.team100.lib.follower;

/**
 * Known-good controller settings.
 */
public class DriveTrajectoryFollowerFactory {

    private final DriveTrajectoryFollowerUtil m_util;

    public DriveTrajectoryFollowerFactory(DriveTrajectoryFollowerUtil util) {
        m_util = util;
    }

    public DrivePIDFLockFollower reefLockedPIDF(DrivePIDFLockFollower.Log log) {
        return new DrivePIDFLockFollower(log, m_util, false, 2.4, 6);
    }

    public DriveTrajectoryFollower fancyPIDF(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 2.4, 1.3);
    }

    public DriveTrajectoryFollower straightPIDF(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 4, 4);
    }

    public DriveTrajectoryFollower newNewPIDF(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 5.5, 4);
    }

    public DriveTrajectoryFollower complementPIDF(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 6, 6);
    }

    public DriveTrajectoryFollower goodPIDF(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 1, 1.3);
    }

    public DriveTrajectoryFollower stageBase(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 2, 1.3);
    }

    public DriveTrajectoryFollower autoPIDF(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 1, 1.3);
    }

    public DriveTrajectoryFollower ffOnly(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, true, 2.4, 1.3);
    }

    public DriveTrajectoryFollower testPIDF(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, false, 2.4, 2.4);
    }

    public DriveTrajectoryFollower testFFOnly(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, true, 2.4, 2.4);
    }

    public DriveTrajectoryFollower fasterCurves(DrivePIDFFollower.Log log) {
        return new DrivePIDFFollower(log, m_util, true, 4.5, 4.5);
    }
}
