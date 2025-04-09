package org.team100.lib.profile.roadrunner;

import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

/** Time-parameterized motion profile. */
public class MotionProfile {
    private final List<MotionSegment> segments;

    /**
     * Trapezoidal motion profile composed of motion segments.
     *
     * @param segments profile motion segments
     */
    public MotionProfile(List<MotionSegment> segments) {
        if (segments.isEmpty())
            throw new IllegalArgumentException();
        this.segments = segments;
    }

    /**
     * Returns the [MotionState] at time [t].
     */
    public MotionState get(double t) {
        if (t < 0.0)
            return segments.get(0).start().stationary();

        var remainingTime = t;
        for (MotionSegment segment : segments) {
            if (remainingTime <= segment.dt()) {
                return segment.get(remainingTime);
            }
            remainingTime -= segment.dt();
        }

        return segments.get(segments.size() - 1).end().stationary();
    }

    /**
     * Returns the duration of the motion profile.
     */
    public double duration() {
        return segments.stream().map((it) -> it.dt()).reduce(0.0, Double::sum);
    }

    /**
     * Returns a reversed version of the motion profile.
     */
    public MotionProfile reversed() {
        List<MotionSegment> l = segments.stream().map((it) -> it.reversed()).collect(Collectors.toList());
        Collections.reverse(l);
        return new MotionProfile(l);
    }

    /**
     * Returns a flipped version of the motion profile.
     */
    public MotionProfile flipped() {
        return new MotionProfile(segments.stream().map((it) -> it.flipped()).collect(Collectors.toList()));
    }

    /**
     * Returns the start [MotionState].
     */
    public MotionState start() {
        return segments.get(0).start();
    }

    /**
     * Returns the end [MotionState].
     */
    public MotionState end() {
        return segments.get(segments.size() - 1).end();
    }

    /**
     * Returns a new motion profile with [other] concatenated.
     */
    MotionProfile append(MotionProfile other) {
        MotionProfileBuilder builder = new MotionProfileBuilder(start());
        builder.appendProfile(this);
        builder.appendProfile(other);
        return builder.build();
    }

    public List<MotionSegment> getSegments() {
        return segments;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("MotionProfile [\n");
        for (MotionSegment s : segments) {
            sb.append(s);
            sb.append("\n");
            sb.append("]");
        }
        return sb.toString();
    }

}