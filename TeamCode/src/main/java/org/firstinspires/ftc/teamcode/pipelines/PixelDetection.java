package org.firstinspires.ftc.teamcode.pipelines;



//import com.acmerobotics.dashboard.config.Config;

import static org.firstinspires.ftc.teamcode.pipelines.BoxDetection.PERCENT_COLOR_THRESHOLD;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

//for dashboard
//@Config
public class PixelDetection extends OpenCvPipeline {
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
    public static int boxes = 60;
    public static int x1 = maxX/3;
    public static int x2 = (maxX * 2)/3;

    static double PERCENT_COLOR_THRESHOLD = 0.4;

    static  Rect LEFT_TARGET = new Rect(
            new Point(0, minY),
            new Point(x1, maxY));
    static  Rect MIDDLE_TARGET = new Rect(
            new Point(x1, minY),
            new Point(x2, maxY));
    static  Rect RIGHT_TARGET = new Rect(
            new Point(x2, minY),
            new Point(960, maxY));
    public static double[] degree = {0};

    double incrementX = (double) maxX/boxes;
    double incrementY = (double) maxY/boxes;
    public PixelDetection(Telemetry t) { telemetry = t; }
    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar low, high;
        low = new Scalar(0, 0, 185);
        high = new Scalar(255, 50, 255);



        Core.inRange(mat, low, high, mat);

        Scalar colorStone = new Scalar(255, 0, 0);
        ArrayList<Double> degrees = new ArrayList<>();
        degrees.clear();
        degree[0] = 0;
        double currentDegree = -90;
        //Adding "boxes" boxes to identify the angle of the junction infront of the camera.
        for (double i = incrementX; i <= maxX; i += incrementX){
            currentDegree += 180.0/boxes;
            Rect temp = new Rect(
                    new Point(i - incrementX, minY),
                    new Point(i, maxY)
            );
            double tempArea = temp.area();
            double threshold = Core.sumElems(mat.submat(temp)).val[0] / tempArea / 255;
            if (threshold > PERCENT_COLOR_THRESHOLD) {
                degrees.add(currentDegree);
                degree[0] += currentDegree;
            }
            Imgproc.rectangle(mat, temp, colorStone);
        }

        if(degree[0] != 0){
            degree[0] /= (double) degrees.size();
        }


        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        telemetry.addData("degree", degree[0]);
        telemetry.update();

        return mat;

    }
}