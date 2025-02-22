package org.team100.lib.async;

import static org.junit.jupiter.api.Assertions.assertEquals;

class AsyncTest {
    private int counter = 0;

    // There's no need to run this all the time
    // it takes a whole second which is a lot in the unit test runner.
    // @Test
    void testSimple() throws InterruptedException {
        Async async = new ExecutorAsync();
        async.addPeriodic(() -> counter += 1, 0.1, "test");
        Thread.sleep(1000); // 1 sec
        assertEquals(9, counter, 1);
    }
}
