package org.firstinspires.ftc.teamcode.pipelines;



//import com.acmerobotics.dashboard.config.Config;

import static java.lang.Thread.sleep;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//for dashboard
//@Config
public class BoxDetection extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT,

    }
    private Location location;

    static final int maxX = 960;
    static final int maxY = 540;
    static final int minY = 300;
    public static int x1 = maxX/2;


    static  Rect MIDDLE_TARGET = new Rect(
            new Point(150, minY),
            new Point(420, maxY));
    static  Rect RIGHT_TARGET = new Rect(
            new Point(700, minY),
            new Point(maxX, maxY));

    static double PERCENT_COLOR_THRESHOLD = 0.1;
    public static int S11 = 100;
    public static int S12 = 40;
    public static int S13 = 55;
    public static int S21 = 125;
    public static int S22 = 150;
    public static int S23 = 95;
    public boolean isRed;
    public BoxDetection(Telemetry t, Rect mid, Rect right, boolean red) {
        telemetry = t;
        MIDDLE_TARGET = mid;
        RIGHT_TARGET = right;
        this.isRed = red;
    }
    public BoxDetection(Telemetry t) {
        telemetry = t;
        this.isRed = false;
    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar low, high;
        if (isRed) {
            low = new Scalar(0, 125, 0);
            high = new Scalar(26, 255, 250);
        }else{
            low = new Scalar(0, 100, 0);
            high = new Scalar(125, 255, 255);
        }


        Core.inRange(mat, low, high, mat);

        Mat middleMat = mat.submat(MIDDLE_TARGET);
        Mat rightMat = mat.submat(RIGHT_TARGET);

        double MIDDLE_VALUE = Core.sumElems(middleMat).val[0] / MIDDLE_TARGET.area() / 255;
        double RIGHT_VALUE = Core.sumElems(rightMat).val[0] / RIGHT_TARGET.area() / 255;

        middleMat.release();
        rightMat.release();

        telemetry.addData("Middle raw value", (int) Core.sumElems(middleMat).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(rightMat).val[0]);
        telemetry.addData("Middle percentage", Math.round(MIDDLE_VALUE * 100) + "%");
        telemetry.addData("Right percentage", Math.round(RIGHT_VALUE * 100) + "%");

        boolean middle = MIDDLE_VALUE > RIGHT_VALUE + .1 && MIDDLE_VALUE > PERCENT_COLOR_THRESHOLD;
        boolean right = RIGHT_VALUE > MIDDLE_VALUE + .1 && RIGHT_VALUE > PERCENT_COLOR_THRESHOLD;


        if (right) {
            location = Location.RIGHT;
            telemetry.addData("Box Detection", "Right");
        }
        else if (middle){
            location = Location.MIDDLE;
            telemetry.addData("Box Detection", "Middle");
        }
        else {
            location = Location.LEFT;
            telemetry.addData("Box Detection", "No Detection, defaulted to Left");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);


        Scalar white = new Scalar(255, 255, 255);

        Imgproc.rectangle(mat, MIDDLE_TARGET, white);
        Imgproc.rectangle(mat, RIGHT_TARGET, white);

        return mat;

    }
    public Location getLocation(){
        return location;
    }
}