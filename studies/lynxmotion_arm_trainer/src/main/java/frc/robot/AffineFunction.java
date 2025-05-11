package frc.robot;

/**
 * Represents an affine transfer function, i.e. scaling and translation. The
 * form should be familiar from eighth grade curriculum:
 * 
 * y = mx + b
 * 
 * Note: there are no units here, make sure you know what you're doing with
 * units.
 */
public class AffineFunction {
    private static final double EPSILON = 1e-6;
    private final double m;
    private final double b;

    public AffineFunction(double m, double b) {
        this.m = m;
        this.b = b;
    }

    public double y(double x) {
        return m * x + b;
    }

    /** The inverse. */
    public double x(double y) {
        if (Math.abs(m) < EPSILON)
            throw new IllegalStateException("slope is zero");
        return (y - b) / m;
    }
}