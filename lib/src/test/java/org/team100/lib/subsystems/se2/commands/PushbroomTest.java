package org.team100.lib.subsystems.se2.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PushbroomTest {
    private static final double DELTA = 0.001;

    @Test
    void testCandidate0() {
        Translation2d target = new Translation2d(0, 0);
        Pose2d robot = new Pose2d(0, 0, Rotation2d.kZero);
        Transform2d offset = new Transform2d(0, 0, Rotation2d.kZero);
        Pose2d candidate = Pushbroom.candidate(new Pose2d(), target, robot, offset);
        assertEquals(0, candidate.getX(), DELTA);
        assertEquals(0, candidate.getY(), DELTA);
        assertEquals(0, candidate.getRotation().getSin(), DELTA);
        assertEquals(1, candidate.getRotation().getCos(), DELTA);
    }

    @Test
    void testCandidate1() {
        Translation2d target = new Translation2d(0, 0);
        Pose2d robot = new Pose2d(0, 0, Rotation2d.kZero);
        Transform2d offset = new Transform2d(0, 0, Rotation2d.k180deg);
        Pose2d candidate = Pushbroom.candidate(new Pose2d(), target, robot, offset);
        assertEquals(0, candidate.getX(), DELTA);
        assertEquals(0, candidate.getY(), DELTA);
        assertEquals(0, candidate.getRotation().getSin(), DELTA);
        assertEquals(-1, candidate.getRotation().getCos(), DELTA);
    }

    @Test
    void testCandidate2() {
        Translation2d target = new Translation2d(1, 0);
        Pose2d robot = new Pose2d(0, 0, Rotation2d.kZero);
        Transform2d offset = new Transform2d(0, 0, Rotation2d.kZero);
        Pose2d candidate = Pushbroom.candidate(new Pose2d(), target, robot, offset);
        assertEquals(1, candidate.getX(), DELTA);
        assertEquals(0, candidate.getY(), DELTA);
        assertEquals(0, candidate.getRotation().getSin(), DELTA);
        assertEquals(1, candidate.getRotation().getCos(), DELTA);
    }

    @Test
    void testCandidate3() {
        Translation2d target = new Translation2d(1, 0);
        Pose2d robot = new Pose2d(0, 0, Rotation2d.kZero);
        Transform2d offset = new Transform2d(0, 0, Rotation2d.k180deg);
        Pose2d candidate = Pushbroom.candidate(new Pose2d(), target, robot, offset);
        assertEquals(1, candidate.getX(), DELTA);
        assertEquals(0, candidate.getY(), DELTA);
        assertEquals(0, candidate.getRotation().getSin(), DELTA);
        assertEquals(-1, candidate.getRotation().getCos(), DELTA);
    }
}
