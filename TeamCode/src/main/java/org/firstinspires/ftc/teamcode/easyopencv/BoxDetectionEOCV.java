package org.firstinspires.ftc.teamcode.easyopencv;



//import com.acmerobotics.dashboard.config.Config;

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
public class BoxDetectionEOCV extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    Mat output = new Mat();
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
                        new Point(440, 250),
                        new Point(580, 406)
                );

    static  Rect  OTHER_TARGET = new Rect(
                        new Point(50, 250),
                        new Point(160, 406)
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
    public BoxDetectionEOCV(Telemetry t, Rect mid, Rect right, boolean red, boolean backdrop) {
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
    public BoxDetectionEOCV(Telemetry t) {
        telemetry = t;
        this.isRed = false;
    }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        input.copyTo(output);
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

        Scalar red = new Scalar(255, 0, 0);
        Scalar green = new Scalar(0,255,0);


        if (other) {
            location = isLeft ? Location.LEFT : Location.RIGHT;
            String position = location == Location.RIGHT ? "Right": "Left";

            telemetry.addData("Box Detection", position);
            Imgproc.rectangle(output, MIDDLE_TARGET, red);
            Imgproc.rectangle(output, OTHER_TARGET, green);
        }
        else if (middle){
            location = Location.MIDDLE;
            telemetry.addData("Box Detection", "Middle");
            Imgproc.rectangle(output, MIDDLE_TARGET, green);
            Imgproc.rectangle(output, OTHER_TARGET, red);

        }
        else {
            location = isLeft ? Location.RIGHT : Location.LEFT;
            String position = location == Location.RIGHT ? "Right": "Left";
            telemetry.addData("Box Detection", position);
            Imgproc.rectangle(output, MIDDLE_TARGET, red);
            Imgproc.rectangle(output, OTHER_TARGET, red);
        }

        telemetry.update();






        return mat;
        //return output;

    }
    public Location getLocation(){
        return location;
    }
}