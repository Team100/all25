package frc.robot;

import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;

import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Use the glass "mechanism" display to show the arm position.
 * 
 * The general idea is to project the link positions into a 2d plane, and then
 * render that.
 * 
 * TODO: also render the workspace (i.e the table).
 */
public class LynxArmVisualizer {
    private final Mechanism2d m_view = new Mechanism2d(100, 100);
    // camera is pointing at the origin in the center: (50,50)
    private final MechanismRoot2d m_root = m_view.getRoot("root", 50, 50);

    public LynxArmVisualizer() {
        OpenCvLoader.forceStaticLoad();
        // base angle is zero (pointing right)
        MechanismLigament2d base = new MechanismLigament2d(
                "link", 0, 0, 5, new Color8Bit(Color.kBlack));
        m_root.append(base);

        MatOfPoint2f points = foo();
        List<Point> pointList = points.toList();
        double x0 = 50;
        double y0 = 50;
        double t0 = 0;
        for (int i = 0; i < pointList.size(); ++i) {
            Point p = pointList.get(i);

            double dx = p.x - x0;
            double dy = p.y - y0;
            double length = Math.hypot(dx, dy);
            double absoluteAngle = Math.atan2(dy, dx);
            double relativeAngle = absoluteAngle - t0;
            MechanismLigament2d link = new MechanismLigament2d(
                    "link" + i, length, Math.toDegrees(relativeAngle), 2, i==0?new Color8Bit(Color.kBlack):new Color8Bit(Color.kWhite));
            base.append(link);
            base = link;
            x0 = p.x;
            y0 = p.y;
            t0 = absoluteAngle;
        }

        SmartDashboard.putData("SideView", m_view);
    }

    public MatOfPoint2f foo() {

        // a cube centered on the origin
        // TODO: this is camera coords, so fix that
        List<Translation3d> tList = List.of(
                new Translation3d(1, 1, 1),
                new Translation3d(-1, 1, 1),
                new Translation3d(-1, -1, 1),
                new Translation3d(1, -1, 1),
                new Translation3d(1, 1, -1),
                new Translation3d(-1, 1, -1),
                new Translation3d(-1, -1, -1),
                new Translation3d(1, -1, -1));

        // the camera view is similar to the lynx board: camera is behind the workspace,
        // a little bit oblique.
        // the camera view is +z
        // Pose3d camera = new Pose3d(0, 0, -4, new Rotation3d(0.3, 0.3, 0));
        Pose3d camera = new Pose3d(-4, 0, 0, new Rotation3d(0, 0.4, 0.3));
        camera = camera.rotateBy(new Rotation3d(MatBuilder.fill(Nat.N3(), Nat.N3(),
                0, -1, 0, //
                0, 0, -1, //
                1, 0, 0)));

        System.out.println("CAMERA " + camera);
        Matrix<N3, N3> r = camera.getRotation().toMatrix();
        Mat rmat = new Mat(3, 3, CvType.CV_64F);
        rmat.put(0, 0, r.get(0, 0));
        rmat.put(0, 1, r.get(0, 1));
        rmat.put(0, 2, r.get(0, 2));
        rmat.put(1, 0, r.get(1, 0));
        rmat.put(1, 1, r.get(1, 1));
        rmat.put(1, 2, r.get(1, 2));
        rmat.put(2, 0, r.get(2, 0));
        rmat.put(2, 1, r.get(2, 1));
        rmat.put(2, 2, r.get(2, 2));
        Mat rvec = new Mat(3, 1, CvType.CV_64F);
        Calib3d.Rodrigues(rmat, rvec);

        // Mat rVec = Mat.zeros(3, 1, CvType.CV_64F);
        Translation3d t = camera.getTranslation();

        Mat tVec = Mat.zeros(3, 1, CvType.CV_64F);
        tVec.put(0, 0, t.getX(), t.getY(), t.getZ());
        // tVec.put(0, 0, 0, 0, -4);

        Mat kMat = Mat.zeros(3, 3, CvType.CV_64F);
        kMat.put(0, 0,
                100.0, 0.0, 50.0,
                0.0, 100.0, 50.0,
                0.0, 0.0, 1.0);

        MatOfDouble dMat = new MatOfDouble(0, 0, 0, 0, 0);
        MatOfPoint2f imagePts2f = new MatOfPoint2f();

        Calib3d.projectPoints(objectPts(tList), rvec, tVec, kMat, dMat, imagePts2f);
        return imagePts2f;
    }

    public static Mat rvec() {
        Mat rVec = Mat.zeros(3, 1, CvType.CV_64F);
        return rVec;
    }

    public static Mat tvec() {
        Mat tVec = Mat.zeros(3, 1, CvType.CV_64F);
        tVec.put(0, 0, 0, 0, -10);
        return tVec;
    }

    public static MatOfPoint3f objectPts(List<Translation3d> tList) {
        // TODO: permute the coordinates
        Point3[] pList = new Point3[tList.size()];
        for (int i = 0; i < tList.size(); ++i) {
            pList[i] = point(tList.get(i));
        }
        return new MatOfPoint3f(pList);
    }

    public static Point3 point(Translation3d t) {
        return new Point3(t.getX(), t.getY(), t.getZ());
    }
}
