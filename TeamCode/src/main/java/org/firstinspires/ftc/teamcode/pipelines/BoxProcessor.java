package org.firstinspires.ftc.teamcode.pipelines;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class BoxProcessor implements VisionProcessor {

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


    static Rect MIDDLE_TARGET = new Rect(
            new Point(520, 356),
            new Point(680, 500)
    );

    static Rect OTHER_TARGET = new Rect(
            new Point(120, 356),
            new Point(300, 520)
    );

    static double PERCENT_COLOR_THRESHOLD = 0.1;
    public static int S11 = 100;
    public static int S12 = 40;
    public static int S13 = 55;
    public static int S21 = 125;
    public static int S22 = 150;
    public static int S23 = 95;
    public boolean isRed;
    public boolean isLeft;

    public BoxProcessor(Telemetry t, Rect mid, Rect right, boolean red, boolean backdrop) {
        telemetry = t;
        MIDDLE_TARGET = mid;
        OTHER_TARGET = right;
        this.isRed = red;
        if (backdrop == true && red == true){
            this.isLeft = false;
        }else if(backdrop == false && red == true){
            this.isLeft = true;
        }else if(backdrop == true){
            this.isLeft = true;
        }else {
            this.isLeft = false;
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if(frame.empty()){
            return Location.MIDDLE;
        }
        Imgproc.cvtColor(frame, mat, Imgproc.COLOR_RGB2HSV);
        Scalar low, high;
        if (isRed) {
            low = new Scalar(0, 125, 0);
            high = new Scalar(26, 255, 250);
        }else{
            low = new Scalar(65, 100, 0);
            high = new Scalar(140, 255, 100);
        }


        Core.inRange(mat, low, high, mat);

        Mat middleMat = mat.submat(MIDDLE_TARGET);
        Mat otherMat = mat.submat(OTHER_TARGET);

        double MIDDLE_VALUE = Core.sumElems(middleMat).val[0] / MIDDLE_TARGET.area() / 255;
        double OTHER_VALUE = Core.sumElems(otherMat).val[0] / OTHER_TARGET.area() / 255;

        middleMat.release();
        otherMat.release();

        telemetry.addData("Middle raw value", (int) Core.sumElems(middleMat).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(otherMat).val[0]);
        telemetry.addData("Middle percentage", Math.round(MIDDLE_VALUE * 100) + "%");
        telemetry.addData("Right percentage", Math.round(OTHER_VALUE * 100) + "%");

        boolean middle = MIDDLE_VALUE > OTHER_VALUE + .1 && MIDDLE_VALUE > PERCENT_COLOR_THRESHOLD;
        boolean other = OTHER_VALUE > MIDDLE_VALUE + .1 && OTHER_VALUE > PERCENT_COLOR_THRESHOLD;


        if (other) {
            location = isLeft ? Location.LEFT : Location.RIGHT;
            String position = location == Location.RIGHT ? "Right": "Left";

            telemetry.addData("Box Detection", position);
        }
        else if (middle){
            location = Location.MIDDLE;
            telemetry.addData("Box Detection", "Middle");
        }
        else {
            location = isLeft ? Location.RIGHT : Location.LEFT;
            String position = location == Location.RIGHT ? "Right": "Left";
            telemetry.addData("Box Detection", position);
        }

        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);


        Scalar white = new Scalar(255, 255, 255);



        return mat;
    }

    public Location getLocation(){
        return location;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
