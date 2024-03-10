package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.pipelines.BoxDetection;
import org.firstinspires.ftc.teamcode.subclasses.Distance;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp(name = "DistanceTesting")
public class DistanceTesting extends LinearOpMode {
    Distance distance;

    Distance frontDistance;

    public static double front_filter = 0.8;
    public static double back_filter = 0.5;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        distance = new Distance(hardwareMap,telemetry);
        frontDistance = new Distance(hardwareMap, telemetry, true);

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            telemetry.addData("Raw Front Distance", frontDistance.getDist());
            telemetry.addData("Filtered Front Distance", frontDistance.getFilteredDist());

            telemetry.addData("Raw Back Distance", distance.getDist());
            telemetry.addData("Filtered Back Distance", distance.getFilteredDist());

            frontDistance.setFilter(front_filter);
            distance.setFilter(back_filter);

            distance.update();
            frontDistance.update();

            distance.telemetry();
            frontDistance.telemetry();
            telemetry.update();
        }
    }

}
