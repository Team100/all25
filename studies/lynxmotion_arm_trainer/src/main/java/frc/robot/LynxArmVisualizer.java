package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.team100.lib.motion.lynxmotion_arm.LynxArmKinematics.LynxArmPose;

import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Use the glass "mechanism" display to show the arm position in 3d.
 */
public class LynxArmVisualizer {
    // Line width in the widget, in pixels.
    private static final int LINE_WIDTH = 3;
    // Where the camera is pointing.
    private static final Translation3d CENTER = new Translation3d(0.25, 0, 0);
    // Reverses the rotation.
    private static final Transform3d FLIP = new Transform3d(
            Pose3d.kZero, new Pose3d(0, 0, 0, new Rotation3d(0, 0, Math.PI)));
    // Transforms the camera z to look along -x
    private static final Transform3d LOOKAT = new Transform3d(
            new Translation3d(),
            new Rotation3d(MatBuilder.fill(Nat.N3(), Nat.N3(),
                    0, -1, 0, //
                    0, 0, -1, //
                    1, 0, 0)))
            .inverse();

    private final Supplier<LynxArmPose> m_arm;

    private final Mechanism2d m_view;
    private final MechanismRoot2d m_root;
    private final MechanismLigament2d m_base;
    private final Map<String, MechanismLigament2d> ligaments = new HashMap<>();

    // coordinates of the camera.
    private double m_cameraRange;
    private double m_pitch;
    private double m_yaw;

    public LynxArmVisualizer(Supplier<LynxArmPose> arm) {
        OpenCvLoader.forceStaticLoad();

        m_arm = arm;

        // size matches the intrinsic matrix
        m_view = new Mechanism2d(100, 100);

        // camera is pointing at the origin in the center: (50, 50)
        m_root = m_view.getRoot("root", 50, 50);

        // base angle is zero (pointing right)
        m_base = new MechanismLigament2d("link", 0, 0, 0, new Color8Bit(Color.kBlack));
        m_root.append(m_base);

        m_cameraRange = 0.866;
        // pitch up to the camera, i.e. it is above the table.
        m_pitch = -0.615;
        // start behind the arm and to the right
        m_yaw = -2.356;

        SmartDashboard.putData("View", m_view);
    }

    public void periodic() {
        // Orbit in yaw.
        // TODO: Also use POV to move around?
        m_yaw += 0.002;
        paintAll();
    }

    Pose3d getCameraPose() {
        // spherical location of the camera; this rotation is not the direction
        // the camera is facing
        Rotation3d r = new Rotation3d(0, m_pitch, m_yaw);
        Translation3d t = new Translation3d(m_cameraRange, 0, 0).rotateBy(r);

        // Rotation facing out
        Pose3d p1 = new Pose3d(t, r);
        // Rotation facing in
        Pose3d p2 = p1.plus(FLIP);
        // Rotate the camera z so it faces in
        Pose3d p3 = p2.plus(LOOKAT);
        // Add an offset to the center of the table
        return new Pose3d(p3.getTranslation().plus(CENTER), p3.getRotation());
    }

    void paintAll() {
        Pose3d m_cameraPose = getCameraPose();

        LynxArmPose p = m_arm.get();
        List<Pose3d> pList = List.of(
                Pose3d.kZero,
                p.p1(),
                p.p2(),
                p.p3(),
                p.p4(),
                p.p5());
        paint(m_base, "actual_arm", m_cameraPose, pList, Color.kOrangeRed);

        // List<Translation3d> tList2 = List.of(
        // new Translation3d(0, 0, 0),
        // new Translation3d(0, 0, 0.07),
        // new Translation3d(0.1, 0, 0.2),
        // new Translation3d(0.25, 0, 0.1),
        // new Translation3d(0.3, 0, 0.05));
        // paint(m_base, "arm", m_cameraPose, tList2, Color.kOrangeRed);

        List<Pose3d> tList3 = List.of(
                new Pose3d(0, 0, 0, Rotation3d.kZero),
                new Pose3d(0, 0.5, 0, Rotation3d.kZero),
                new Pose3d(0.5, 0.5, 0, Rotation3d.kZero),
                new Pose3d(0.5, -0.5, 0, Rotation3d.kZero),
                new Pose3d(0, -0.5, 0, Rotation3d.kZero),
                new Pose3d(0, 0, 0, Rotation3d.kZero));
        paint(m_base, "tabletop", m_cameraPose, tList3, Color.kGray);
    }

    /** TODO: render the joint axes too */
    void paint(
            MechanismLigament2d base,
            String name,
            Pose3d cameraPose,
            List<Pose3d> tList,
            Color color) {

        MatOfPoint2f points = project(cameraPose, tList);
        List<Point> pointList = points.toList();
        // System.out.printf("pointlist size %d\n", pointList.size());
        double x0 = 50;
        double y0 = 50;
        double t0 = 0;
        for (int i = 0; i < pointList.size(); ++i) {
            // this is a point in the camera, which is x-left, y-down,
            // looking towards +z,
            // which implies clockwise-positive angles in the xy plane.
            // the mechanism2d widget uses counterclockwise-positive angles.
            // we could invert the xy coordinates, but then we'd have to remember
            // that the "camera" coordinates are unusual.
            // instead, we'll just invert the angle.
            Point p = pointList.get(i);

            double dx = p.x - x0;
            double dy = p.y - y0;
            double length = Math.hypot(dx, dy);
            double absoluteAngle = -1.0 * Math.atan2(dy, dx);
            double relativeAngle = absoluteAngle - t0;
            String fullname = String.format("%s_%d", name, i);
            MechanismLigament2d link = ligaments.get(fullname);
            if (link == null) {
                link = new MechanismLigament2d(
                        fullname,
                        length,
                        Math.toDegrees(relativeAngle),
                        LINE_WIDTH,
                        i == 0 ? new Color8Bit(Color.kBlack) : new Color8Bit(color));
                base.append(link);
                base = link;
                ligaments.put(fullname, link);
            } else {
                link.setLength(length);
                link.setAngle(Math.toDegrees(relativeAngle));
            }
            x0 = p.x;
            y0 = p.y;
            t0 = absoluteAngle;
        }
    }

    MatOfPoint2f project(Pose3d cameraPose, List<Pose3d> tList) {
        // the extrinsic matrix is the inverse of the camera pose.
        Transform3d extrinsic = new Transform3d(Pose3d.kZero, cameraPose).inverse();

        Mat rvec = getRvec(extrinsic);
        Mat tVec = getTVec(extrinsic);
        Mat kMat = getKMat();

        MatOfDouble dMat = new MatOfDouble(0, 0, 0, 0, 0);
        MatOfPoint2f imagePts2f = new MatOfPoint2f();

        Calib3d.projectPoints(objectPts(tList), rvec, tVec, kMat, dMat, imagePts2f);
        return imagePts2f;
    }

    private Mat getTVec(Transform3d extrinsic) {
        Translation3d t = extrinsic.getTranslation();
        Mat tVec = Mat.zeros(3, 1, CvType.CV_64F);
        tVec.put(0, 0, t.getX(), t.getY(), t.getZ());
        return tVec;
    }

    private Mat getKMat() {
        Mat kMat = Mat.zeros(3, 3, CvType.CV_64F);
        kMat.put(0, 0,
                100.0, 0.0, 50.0,
                0.0, 100.0, 50.0,
                0.0, 0.0, 1.0);
        return kMat;
    }

    private Mat getRvec(Transform3d extrinsic) {
        Matrix<N3, N3> r = extrinsic.getRotation().toMatrix();
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
        return rvec;
    }

    static MatOfPoint3f objectPts(List<Pose3d> tList) {
        List<Point3> pList = new ArrayList<Point3>();
        for (Pose3d p : tList) {
            Translation3d t = p.getTranslation();
            pList.add(point(t));
            // also add a line representing the y axis
            Translation3d yt = new Translation3d(0, 1, 0);
            Translation3d a2 = new Translation3d(yt.rotateBy(p.getRotation()).toVector()).times(0.03);
            pList.add(point(t.plus(a2)));
            pList.add(point(t.minus(a2)));
            // and reiterate the joint point so that the links look right
            pList.add(point(t));
        }
        return new MatOfPoint3f(pList.toArray(new Point3[0]));
    }

    static Point3 point(Translation3d t) {
        return new Point3(t.getX(), t.getY(), t.getZ());
    }
}
