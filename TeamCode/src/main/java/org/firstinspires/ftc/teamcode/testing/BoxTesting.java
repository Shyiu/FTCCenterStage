package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.pipelines.BoxDetection;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp(name = "BoxPipelineTesting")
public class BoxTesting extends LinearOpMode {
    BoxDetection pipeline;
    OpenCvCamera camera;
    MecanumBotConstant mc;



    public static Rect MIDDLE_TARGET = new Rect(
            new Point(150, 300),
            new Point(420, 540));
    //(60,404), (230, 544) BLUE STACK and RED BACKDROP
    //(220,404), (440, 544) BLUE BACKDROP AND RED STACK
    public static Rect RIGHT_TARGET = new Rect(
            new Point(700, 300),
            new Point(960, 540));
    public static boolean red = false;
    public static boolean backdrop = false;
    //(450,404), (640, 544) BLUE STACK and RED BACKDROP
    //(640,404), (900, 544) BLUE BACKDROP AND RED STACK
    @Override
    public void runOpMode() throws InterruptedException {
        mc = new MecanumBotConstant();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pipeline = new BoxDetection(telemetry, MIDDLE_TARGET, RIGHT_TARGET, red, backdrop);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, mc.camera), cameraMonitorViewId);
        pipeline.isRed = red;
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(960, 544, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 5);

            }

            @Override
            public void onError(int errorCode) {

            }
        });
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            telemetry.addData("Box Detection", pipeline.getLocation());
            telemetry.update();
        }
    }

}
