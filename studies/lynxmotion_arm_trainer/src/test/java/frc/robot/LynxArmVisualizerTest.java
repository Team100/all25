package frc.robot;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

public class LynxArmVisualizerTest {
    @Test
    void testFoo() {
        LynxArmVisualizer foo = new LynxArmVisualizer();
        MatOfPoint2f points = foo.foo();
        List<Point> pointList = points.toList();
        System.out.printf("len %d\n", pointList.size());
        for (Point p : pointList) {
            System.out.printf("%5.3f, %5.3f\n", p.x, p.y);
        }
    }

}
