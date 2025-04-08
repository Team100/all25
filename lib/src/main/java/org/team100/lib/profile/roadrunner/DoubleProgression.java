package org.team100.lib.profile.roadrunner;

import java.util.Iterator;
import java.util.stream.IntStream;

// TODO: delete this, use a for loop.
public class DoubleProgression implements Iterable<Double> {
    private final double start;
    private final double step;
    private final int size;

    /**
     * A progression of values of type `Double`.
     */
    public DoubleProgression(
            double start,
            double step,
            int size) {
        this.start = start;
        this.step = step;
        this.size = size;
    }

    public static DoubleProgression fromClosedInterval(double start, double endInclusive, int count) {
        double step = 0;
        if (count == 0) {
            step = 0.0;
        } else if (count == 1) {
            step = 1.0;
        } else {
            step = (endInclusive - start) / (count - 1);
        }
        return new DoubleProgression(start, step, count);
    }

    public DoubleProgression plus(double offset) {
        return new DoubleProgression(start + offset, step, size);
    }


    public double get(int index) {
        return start + step * index;
    }

    public int size() {
        return size;
    }

    /**
     * Iterator implementation for [DoubleProgression].
     */
    class IteratorImpl implements Iterator<Double> {
        private final Iterator<Integer> iterator;

        public IteratorImpl() {
            iterator = IntStream.range(0, size - 1).iterator();
        }

        public boolean hasNext() {
            return iterator.hasNext();
        }

        public Double next() {
            return DoubleProgression.this.get(iterator.next());
        }
    }

    public Iterator<Double> iterator() {
        return new IteratorImpl();
    }

    public double getStep() {
        return step;
    }

}