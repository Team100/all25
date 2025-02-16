package org.team100.lib.logging.primitive;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Takt;
import org.team100.lib.util.Util;

class UdpPrimitiveProtocol2PerformanceTest {
    private static final boolean DEBUG = false;

    /**
     * Encoding itself is quite fast, 4 ns per key, about 1 ns
     * per int on my machine, which is about as fast as it can go, I think.
     */
    @Test
    void testEncodingPerformance() throws Exception {
        // final int ITERATIONS = 200000000;
        final int ITERATIONS = 20000000;
        final int BUFFER_SIZE = 10000000;

        // huge buffer for this test; we're testing the encoding.
        UdpPrimitiveProtocol p = new UdpPrimitiveProtocol(BUFFER_SIZE);

        {
            p.clear();
            double t1 = Takt.actual();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putBoolean(17, true))
                    p.clear();
            }
            double t2 = Takt.actual();
            if (DEBUG) {
                Util.printf("boolean duration sec %.3f\n", t2 - t1);
                Util.printf("boolean duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
            }
        }

        {
            p.clear();
            double t1 = Takt.actual();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putDouble(17, 1.0))
                    p.clear();
            }
            double t2 = Takt.actual();
            if (DEBUG) {
                Util.printf("double duration sec %.3f\n", t2 - t1);
                Util.printf("double duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
            }
        }
        {
            p.clear();
            double t1 = Takt.actual();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putInt(17, 1))
                    p.clear();
            }
            double t2 = Takt.actual();
            if (DEBUG) {
                Util.printf("int duration sec %.3f\n", t2 - t1);
                Util.printf("int duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
            }
        }
        {
            p.clear();
            double t1 = Takt.actual();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putDoubleArray(17, new double[] { 1.0 }))
                    p.clear();
            }
            double t2 = Takt.actual();
            if (DEBUG) {
                Util.printf("double[] duration sec %.3f\n", t2 - t1);
                Util.printf("double[] duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
            }
        }
        {
            p.clear();
            double t1 = Takt.actual();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putLong(17, 1))
                    p.clear();
            }
            double t2 = Takt.actual();
            if (DEBUG) {
                Util.printf("long duration sec %.3f\n", t2 - t1);
                Util.printf("long duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
            }
        }
        {
            p.clear();
            double t1 = Takt.actual();
            for (int i = 0; i < ITERATIONS; ++i) {
                if (!p.putString(17, "asdf"))
                    p.clear();
            }
            double t2 = Takt.actual();
            if (DEBUG) {
                Util.printf("string duration sec %.3f\n", t2 - t1);
                Util.printf("string duration per row us %.3f\n", 1000000 * (t2 - t1) / (ITERATIONS));
            }
        }
    }
}
