package org.team100.lib.logging.receiver;

import static java.util.concurrent.TimeUnit.SECONDS;

import java.util.Arrays;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.atomic.AtomicInteger;

import org.team100.lib.logging.primitive.UdpType;
import org.team100.lib.util.Util;

/** For testing */
public class DummyUdpConsumers implements UdpConsumersInterface {

    private static final boolean PRINT = false;
    private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);
    AtomicInteger counter = new AtomicInteger(0);

    public DummyUdpConsumers() {
        if (PRINT)
            Util.println("using dummy consumer");
        scheduler.scheduleAtFixedRate(
                () -> Util.printf("counter %d\n", counter.getAndSet(0)),
                0, 1, SECONDS);
    }

    @Override
    public boolean validateTimestamp(long timestamp) {
        return true;
    }

    @Override
    public void acceptBoolean(int key, boolean val) {
        counter.incrementAndGet();
        if (PRINT)
            Util.printf("bool key: %d value: %b\n", key, val);
    }

    @Override
    public void acceptDouble(int key, double val) {
        counter.incrementAndGet();
        if (PRINT)
            Util.printf("double key: %d value: %f\n", key, val);
    }

    @Override
    public void acceptInt(int key, int val) {
        counter.incrementAndGet();
        if (PRINT)
            Util.printf("int key: %d value: %d\n", key, val);
    }

    @Override
    public void acceptDoubleArray(int key, double[] val) {
        counter.incrementAndGet();
        if (PRINT)
            Util.printf("double[] key: %d value: %s\n", key, Arrays.toString(val));
    }

    @Override
    public void acceptString(int key, String val) {
        counter.incrementAndGet();
        if (PRINT)
            Util.printf("string key: %d value: %s\n", key, val);
    }

    @Override
    public void acceptMeta(int key, UdpType type, String val) {
        counter.incrementAndGet();
        if (PRINT)
            Util.printf("META key: %d type: %s, value: %s\n", key, type.name(), val);
    }

    @Override
    public void flush() {
        if (PRINT)
            Util.printf("udp consumer flushing");
    }

    @Override
    public void close() {
        //
    }

}
