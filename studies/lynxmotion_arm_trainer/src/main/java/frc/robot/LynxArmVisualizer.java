package frc.robot;

import java.util.List;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;

import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    public LynxArmVisualizer() {
        OpenCvLoader.forceStaticLoad();

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

        // the camera view is similar to the lynx board: camera is behind the workspace.
        // these are wpi (x-fwd) coords, in meters
        Pose3d camera = new Pose3d(-0.5, 0, 0.5, new Rotation3d(0, Math.PI / 4, 0));
        Mat rVec = Mat.zeros(3, 1, CvType.CV_64F);
        Mat tVec = Mat.zeros(3, 1, CvType.CV_64F);
        tVec.put(0, 0, 0, 0, -10);
        Mat kMat = Mat.zeros(3, 3, CvType.CV_64F);
        kMat.put(0, 0,
                100.0, 0.0, 50.0,
                0.0, 100.0, 50.0,
                0.0, 0.0, 1.0);

        MatOfDouble dMat = new MatOfDouble(0, 0, 0, 0, 0);
        MatOfPoint2f imagePts2f = new MatOfPoint2f();

        Calib3d.projectPoints(objectPts(tList), rVec, tVec, kMat, dMat, imagePts2f);
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
