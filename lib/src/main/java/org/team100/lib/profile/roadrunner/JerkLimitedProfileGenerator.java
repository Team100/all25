package org.team100.lib.profile.roadrunner;

import java.util.List;

import org.team100.lib.util.Math100;

/**
 * Motion profile generator with arbitrary start and end motion states and
 * jerk limiting.
 */
public class JerkLimitedProfileGenerator {
    public static MotionProfile generateMotionProfile(
            MotionState start,
            MotionState goal,
            double maxVel,
            double maxAccel,
            double maxJerk,
            boolean overshoot) {
        if (goal.getX() < start.getX()) {
            // ensure the goal is always after the start; plan the flipped profile otherwise
            return generateMotionProfile(start.flipped(), goal.flipped(), maxVel, maxAccel, maxJerk, overshoot)
                    .flipped();
        }

        return sCurve(start, goal, maxVel, maxAccel, maxJerk, overshoot);
    }

    //////////////////////////////////////////////////

    /** Jerk-limited profile (S-curve) with finite jerk limit. */
    private static MotionProfile sCurve(
            MotionState start,
            MotionState goal,
            double maxVel,
            double maxAccel,
            double maxJerk,
            boolean overshoot) {
        MotionProfile accelerationProfile = generateAccelProfile(start, maxVel, maxAccel, maxJerk);
        // decel is reversed accel with the goal accel flipped
        MotionProfile decelerationProfile = generateAccelProfile(
                new MotionState(
                        goal.getX(),
                        goal.getV(),
                        -goal.getA(),
                        goal.getJ()),
                maxVel,
                maxAccel,
                maxJerk)
                .reversed();

        MotionProfile noCoastProfile = accelerationProfile.append(decelerationProfile);
        double remainingDistance = goal.getX() - noCoastProfile.end().getX();

        if (remainingDistance >= 0.0) {
            // we just need to add a constant-velocity segment of appropriate duration
            double deltaT4 = remainingDistance / maxVel;

            return new MotionProfileBuilder(start)
                    .appendProfile(accelerationProfile)
                    .appendJerkSegment(0.0, deltaT4)
                    .appendProfile(decelerationProfile)
                    .build();
        }

        // the profile never reaches maxV
        // thus, we need to compute the peak velocity (0 < peak vel < max vel)
        // we *could* construct a large polynomial expression (i.e., a nasty cubic) and
        // solve it using Cardano's
        // method, some kind of inclusion method like modified Anderson-Bjorck-King, or
        // a host of other methods
        // (see https://link.springer.com/content/pdf/bbm%3A978-3-642-05175-3%2F1.pdf
        // for modified ABK)
        // instead, however, we conduct a binary search as it's sufficiently performant
        // for this use case,
        // requires less code, and is overall significantly more comprehensible
        double upperBound = maxVel;
        double lowerBound = 0.0;
        int iterations = 0;
        while (iterations < 1000) {
            double peakVel = (upperBound + lowerBound) / 2;

            MotionProfile searchAccelProfile = generateAccelProfile(start, peakVel, maxAccel, maxJerk);
            MotionProfile searchDecelProfile = generateAccelProfile(goal, peakVel, maxAccel, maxJerk)
                    .reversed();

            MotionProfile searchProfile = searchAccelProfile.append(searchDecelProfile);

            double error = goal.getX() - searchProfile.end().getX();

            if (Math.abs(error) < 1e-6) {
                return searchProfile;
            }

            if (error > 0.0) {
                // we undershot so shift the lower bound up
                lowerBound = peakVel;
            } else {
                // we overshot so shift the upper bound down
                upperBound = peakVel;
            }

            iterations++;
        }

        // constraints are not satisfiable
        if (overshoot) {
            return noCoastProfile.append(
                    generateMotionProfile(
                            noCoastProfile.end(),
                            goal,
                            maxVel,
                            maxAccel,
                            maxJerk,
                            true));
        }
        // violate max jerk first
        return TrapezoidProfileGenerator.generateSimpleMotionProfile(start, goal, maxVel, maxAccel, false);
    }

    /**
     * Returns a profile with two (jerk-limited) or three (jerk-limited,
     * acceleration-limited, jerk-limited) segments to full velocity. The end state
     * has zero acceleration.
     */
    static MotionProfile generateAccelProfile(
            MotionState start,
            double maxVel,
            double maxAccel,
            double maxJerk) {

        // compute the duration and velocity of the first segment
        double deltaT1;
        double deltaV1;
        if (start.getA() > maxAccel) {
            // slow down and see where we are
            deltaT1 = (start.getA() - maxAccel) / maxJerk;
            deltaV1 = start.getA() * deltaT1 - 0.5 * maxJerk * deltaT1 * deltaT1;
        } else {
            // otherwise accelerate
            deltaT1 = (maxAccel - start.getA()) / maxJerk;
            deltaV1 = start.getA() * deltaT1 + 0.5 * maxJerk * deltaT1 * deltaT1;
        }

        // compute the duration and velocity of the third segment
        double deltaT3 = maxAccel / maxJerk;
        double deltaV3 = maxAccel * deltaT3 - 0.5 * maxJerk * deltaT3 * deltaT3;

        // compute the velocity change required in the second segment
        double deltaV2 = maxVel - start.getV() - deltaV1 - deltaV3;

        if (deltaV2 < 0.0) {
            // there is no constant acceleration phase
            // the second case checks if we're going to exceed max vel
            if (start.getA() > maxAccel
                    || (start.getV() - maxVel) > (start.getA() * start.getA()) / (2 * maxJerk)) {
                // problem: we need to cut down on our acceleration but we can't cut our initial
                // decel
                // solution: we'll lengthen our initial decel to -max accel and similarly with
                // our final accel
                // if this results in an over correction, decel instead to a good accel
                double newDeltaT1 = (start.getA() + maxAccel) / maxJerk;
                double newDeltaV1 = start.getA() * newDeltaT1 - 0.5 * maxJerk * newDeltaT1 * newDeltaT1;

                double newDeltaV2 = maxVel - start.getV() - newDeltaV1 + deltaV3;

                if (newDeltaV2 > 0.0) {
                    // we decelerated too much
                    List<Double> roots = Math100.solveQuadratic(
                            -maxJerk,
                            2 * start.getA(),
                            start.getV() - maxVel - start.getA() * start.getA() / (2 * maxJerk));
                    double finalDeltaT1 = roots.stream().filter((it) -> it >= 0.0).min(Double::compare)
                            .orElseThrow();
                    double finalDeltaT3 = finalDeltaT1 - start.getA() / maxJerk;

                    return new MotionProfileBuilder(start)
                            .appendJerkSegment(-maxJerk, finalDeltaT1)
                            .appendJerkSegment(maxJerk, finalDeltaT3)
                            .build();
                } else {
                    // we're almost good
                    double newDeltaT2 = newDeltaV2 / -maxAccel;

                    return new MotionProfileBuilder(start)
                            .appendJerkSegment(-maxJerk, newDeltaT1)
                            .appendJerkSegment(0.0, newDeltaT2)
                            .appendJerkSegment(maxJerk, deltaT3)
                            .build();
                }
            } else {
                // cut out the constant accel phase and find a shorter delta t1 and delta t3
                List<Double> roots = Math100.solveQuadratic(
                        maxJerk,
                        2 * start.getA(),
                        start.getV() - maxVel + start.getA() * start.getA() / (2 * maxJerk));
                double newDeltaT1 = roots.stream().filter((it) -> it >= 0.0).min(Double::compare).orElseThrow();
                double newDeltaT3 = newDeltaT1 + start.getA() / maxJerk;

                return new MotionProfileBuilder(start)
                        .appendJerkSegment(maxJerk, newDeltaT1)
                        .appendJerkSegment(-maxJerk, newDeltaT3)
                        .build();
            }
        } else {
            // there is a constant acceleration phase
            double deltaT2 = deltaV2 / maxAccel;

            MotionProfileBuilder builder = new MotionProfileBuilder(start);
            if (start.getA() > maxAccel) {
                builder.appendJerkSegment(-maxJerk, deltaT1);
            } else {
                builder.appendJerkSegment(maxJerk, deltaT1);
            }
            return builder.appendJerkSegment(0.0, deltaT2)
                    .appendJerkSegment(-maxJerk, deltaT3)
                    .build();
        }

    }

}
