package frc.robot;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class LynxArmVisualizerTest {
    @Test
    void testFoo() {
        try (LynxArm arm = new LynxArm()) {
            LynxArmVisualizer foo = new LynxArmVisualizer(arm);
            // unrotated camera below the table
            Pose3d cameraPose = new Pose3d(0, 0, -4, new Rotation3d(0, 0, 0));
            // a cube centered on the origin
            List<Translation3d> tList = List.of(
                    new Translation3d(1, 1, 1),
                    new Translation3d(-1, 1, 1),
                    new Translation3d(-1, -1, 1),
                    new Translation3d(1, -1, 1),
                    new Translation3d(1, 1, -1),
                    new Translation3d(-1, 1, -1),
                    new Translation3d(-1, -1, -1),
                    new Translation3d(1, -1, -1));
            MatOfPoint2f points = foo.project(cameraPose, tList);
            List<Point> pointList = points.toList();
            System.out.printf("pointList.size() = %d\n", pointList.size());
            for (Point p : pointList) {
                System.out.printf("%5.3f, %5.3f\n", p.x, p.y);
            }
        }
    }

}
