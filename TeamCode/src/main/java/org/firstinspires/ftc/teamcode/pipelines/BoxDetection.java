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
    static final int minY = 120;
    public static int x1 = maxX/3;
    public static int x2 = (maxX * 2)/3;

    static  Rect LEFT_TARGET = new Rect(
            new Point(0, minY),
            new Point(x1, maxY));
    static  Rect MIDDLE_TARGET = new Rect(
            new Point(x1, minY),
            new Point(x2, maxY));
    static  Rect RIGHT_TARGET = new Rect(
            new Point(x2, minY),
            new Point(960, maxY));

    static double PERCENT_COLOR_THRESHOLD = 0.4;
    public static int S11 = 75;
    public static int S12 = 40;
    public static int S13 = 0;
    public static int S21 = 125;
    public static int S22 = 255;
    public static int S23 = 100;
    public boolean red = true;
    public BoxDetection(Telemetry t) { telemetry = t; }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar low, high;
        if (red) {
            low = new Scalar(0, 180, 0);
            high = new Scalar(50, 250, 250);
        }else{
            low = new Scalar(S11, S12, S13);
            high = new Scalar(S21, S22, S23);
        }


        Core.inRange(mat, low, high, mat);

        Mat leftMat = mat.submat(LEFT_TARGET);
        Mat middleMat = mat.submat(MIDDLE_TARGET);
        Mat rightMat = mat.submat(RIGHT_TARGET);

        double LEFT_VALUE = Core.sumElems(leftMat).val[0] / LEFT_TARGET.area() / 255;
        double MIDDLE_VALUE = Core.sumElems(middleMat).val[0] / MIDDLE_TARGET.area() / 255;
        double RIGHT_VALUE = Core.sumElems(rightMat).val[0] / RIGHT_TARGET.area() / 255;

        rightMat.release();
        leftMat.release();
        middleMat.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(leftMat).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middleMat).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(rightMat).val[0]);
        telemetry.addData("Left percentage", Math.round(LEFT_VALUE * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(MIDDLE_VALUE * 100) + "%");
        telemetry.addData("Right percentage", Math.round(RIGHT_VALUE * 100) + "%");

        boolean left = LEFT_VALUE > MIDDLE_VALUE + .1 && LEFT_VALUE > RIGHT_VALUE + .1;
        boolean middle = MIDDLE_VALUE > LEFT_VALUE + .1 && MIDDLE_VALUE > RIGHT_VALUE + .1;
        boolean right = RIGHT_VALUE > LEFT_VALUE + .1 && RIGHT_VALUE > MIDDLE_VALUE + .1;


        if (left) {
            location = Location.LEFT;
            telemetry.addData("Box Detection", "Left");
        }
        else if (right){
            location = Location.RIGHT;
            telemetry.addData("Box Detection", "Right");
        }
        else if(middle){
            location = Location.MIDDLE;
            telemetry.addData("Box Detection", "Middle");
        }else{
            location = Location.MIDDLE;
            telemetry.addData("Box Detection", "No Detection, defaulted to middle");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);


        Scalar white = new Scalar(255, 255, 255);

        Imgproc.rectangle(mat, LEFT_TARGET, white);
        Imgproc.rectangle(mat, MIDDLE_TARGET, white);
        Imgproc.rectangle(mat, RIGHT_TARGET, white);

        return mat;

    }
}