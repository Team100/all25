package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/**
 * https://docs.google.com/spreadsheets/d/1XCtQGnJABVWTuCkx0t6u_xJjezelqyBE2Rz7eqmoyh4
 */
public class CalibrationTest {
    // LINEST provides the standard error of the prediction
    private static final double STD_ERR_Y = 0.0286;
    // allow 2 std dev
    private static final double z = 1.5;
    // this is just a guess
    private static final double ERR_X = 0.015;

    @Test
    void testSwingForward() {
        double m = -3.216;
        double b = 1.534;
        AffineFunction f = new AffineFunction(m, b);
        // these are the points I used

        assertEquals(1.571, f.y(0), z*STD_ERR_Y);
        assertEquals(1.239, f.y(0.1), z*STD_ERR_Y);
        assertEquals(0.873, f.y(0.2), z*STD_ERR_Y);
        assertEquals(0.541, f.y(0.3), z*STD_ERR_Y);
        assertEquals(0.244, f.y(0.4), z*STD_ERR_Y);
        assertEquals(-0.105, f.y(0.5), z*STD_ERR_Y);
        assertEquals(-0.419, f.y(0.6), z*STD_ERR_Y);
        assertEquals(-0.716, f.y(0.7), z*STD_ERR_Y);
        assertEquals(-1.065, f.y(0.8), z*STD_ERR_Y);
        assertEquals(-1.344, f.y(0.9), z*STD_ERR_Y);
        assertEquals(-1.641, f.y(1.0), z*STD_ERR_Y);
    }

    @Test
    void testSwingInverse() {
        double m = -3.216;
        double b = 1.534;
        AffineFunction f = new AffineFunction(m, b);
        // these are the points I used

        assertEquals(0.0, f.x(1.571), ERR_X);
        assertEquals(0.1, f.x(1.239), ERR_X);
        assertEquals(0.2, f.x(0.873), ERR_X);
        assertEquals(0.3, f.x(0.541), ERR_X);
        assertEquals(0.4, f.x(0.244), ERR_X);
        assertEquals(0.5, f.x(-0.105), ERR_X);
        assertEquals(0.6, f.x(-0.419), ERR_X);
        assertEquals(0.7, f.x(-0.716), ERR_X);
        assertEquals(0.8, f.x(-1.065), ERR_X);
        assertEquals(0.9, f.x(-1.344), ERR_X);
        assertEquals(1.0, f.x(-1.641), ERR_X);
    }
}
