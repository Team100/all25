package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class AffineFunctionTest {
    @Test
    void testRoundTrip() {
        AffineFunction f = new AffineFunction(-3.216, 1.534);
        double y = f.y(0.0);
        double x = f.x(y);
        assertEquals(0.0, x, 1e-12);
    }
}
