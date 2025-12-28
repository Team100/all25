package org.team100.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.geometry.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class TimedStateTest {
    private static final double EPSILON = 1e-12;

    @Test
    void test() {
        // At (0,0,0), t=0, v=0, acceleration=1
        TimedState start_state = new TimedState(
                new PathPoint(
                        WaypointSE2.irrotational(
                                new Pose2d(0, 0, new Rotation2d(0)), 0, 1.2),
                        0, 0),
                0.0, 0.0, 1.0);

        // At (.5,0,0), t=1, v=1, acceleration=0
        TimedState end_state = new TimedState(
                new PathPoint(
                        WaypointSE2.irrotational(
                                new Pose2d(0.5, 0, new Rotation2d(0)), 0, 1.2),
                        0, 0),
                1.0, 1.0, 0.0);

        // endpoints
        assertEquals(start_state, start_state.interpolate(end_state, 0.0));
        assertEquals(end_state, start_state.interpolate(end_state, 1.0));

        // halfway between the states by time
        TimedState intermediate_state = start_state.interpolate(end_state, 0.5);
        assertEquals(0.5, intermediate_state.getTimeS(), EPSILON);
        assertEquals(start_state.acceleration(), intermediate_state.acceleration(), EPSILON);
        assertEquals(0.5, intermediate_state.velocityM_S(), EPSILON);
        // close to the start state by distance
        assertEquals(0.125, intermediate_state.point().waypoint().pose().getTranslation().getX(), EPSILON);
    }
}
