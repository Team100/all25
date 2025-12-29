package org.team100.frc2025.CalgamesArm;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.r1.CompleteProfile;
import org.team100.lib.profile.r1.CurrentLimitedExponentialProfile;
import org.team100.lib.subsystems.prr.EAWConfig;
import org.team100.lib.subsystems.prr.commands.FollowJointProfiles;

public class MechProfiles {

    public static FollowJointProfiles WithCurrentLimitedExponentialProfile(
            CalgamesMech subsystem, EAWConfig goal) {
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CurrentLimitedExponentialProfile(2, 4, 5), // elevator
                new CurrentLimitedExponentialProfile(8, 8, 16), // arm
                new CurrentLimitedExponentialProfile(8, 8, 16)); // wrist
    }

    /**
     * Accelerate gently but decelerate firmly.
     * 
     * This is for paths that start with lots of gravity torque and end
     * without very much gravity torque, e.g. from "pick" to "home".
     */
    public static FollowJointProfiles slowFast(LoggerFactory parent, CalgamesMech subsystem, EAWConfig goal) {
        LoggerFactory log = parent.name("slowFast");
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CompleteProfile(log.name("elevator"), 2, 4, 6, 5, 50, 50, 0.001), // elevator
                new CompleteProfile(log.name("arm"), 12, 8, 16, 16, 50, 50, 0.001), // arm
                new CompleteProfile(log.name("wrist"), 8, 4, 12, 16, 50, 50, 0.001)); // wrist
    }

    /**
     * Accelerate firmly but decelerate gently.
     * 
     * This is for paths that start without gravity torque but end with a lot of
     * gravity torque, e.g. from "home" to "pick".
     */

    public static FollowJointProfiles fastSlow(LoggerFactory parent, CalgamesMech subsystem, EAWConfig goal) {
        LoggerFactory log = parent.name("fastSlow");
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CompleteProfile(log.name("elevator"), 2, 6, 4, 5, 50, 50, 0.001), // elevator
                new CompleteProfile(log.name("arm"), 12, 20, 8, 16, 100, 100, 0.001), // arm
                new CompleteProfile(log.name("wrist"), 8, 12, 4, 16, 50, 50, 0.001)); // wrist
    }

    public static FollowJointProfiles algae(LoggerFactory parent, CalgamesMech subsystem, EAWConfig goal) {
        LoggerFactory log = parent.name("algae");
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CompleteProfile(log.name("elevator"), 2, 6, 4, 5, 50, 50, 0.001), // elevator
                new CompleteProfile(log.name("arm"), 12, 8, 8, 16, 100, 100, 0.001), // arm
                new CompleteProfile(log.name("wrist"), 8, 12, 4, 16, 50, 50, 0.001)); // wrist
    }

    public static FollowJointProfiles gentle(LoggerFactory parent, CalgamesMech subsystem, EAWConfig goal) {
        LoggerFactory log = parent.name("gentle");
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CompleteProfile(log.name("elevator"), 2, 6, 4, 5, 50, 50, 0.001), // elevator
                new CompleteProfile(log.name("arm"), 2, 2, 2, 8, 100, 100, 0.001), // arm
                new CompleteProfile(log.name("wrist"), 8, 12, 4, 16, 50, 50, 0.001)); // wrist
    }

    public static FollowJointProfiles algaeUp(LoggerFactory parent, CalgamesMech subsystem, EAWConfig goal) {
        LoggerFactory log = parent.name("algaeUp");
        return new FollowJointProfiles(
                subsystem,
                goal,
                new CompleteProfile(log.name("elevator"), 2, 6, 4, 5, 50, 50, 0.001), // elevator
                new CompleteProfile(log.name("arm"), 4, 4, 4, 8, 100, 100, 0.001), // arm
                new CompleteProfile(log.name("wrist"), 8, 12, 4, 16, 50, 50, 0.001)); // wrist
    }

}
