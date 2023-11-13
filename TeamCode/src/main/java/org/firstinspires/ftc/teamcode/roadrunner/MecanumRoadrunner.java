package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.pipelines.BoxDetection;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous
public class MecanumRoadrunner extends LinearOpMode {
    MecanumBotConstant mc;
    OpenCvCamera camera;
    BoxDetection boxDetection;
    SampleMecanumDrive drive;
    TrajectorySequence middle, left, right;
    enum START{
        RED_BACKDROP,
        RED_STACK,
        BLUE_BACKDROP,
        BLUE_STACK
    }
    START position;
    private boolean exit = false;
    @Override
    public void runOpMode(){
        telemetry.addLine("Press x for Blue, y for red");
        telemetry.update();
        while (true) {
            while (!gamepad1.x || !gamepad1.y) {
                if (gamepad1.x) {
                    telemetry.clear();
                    telemetry.addLine("Press x for backstage side, y for stack side");
                    telemetry.update();
                    while (!gamepad1.x || !gamepad1.y) {
                        if (gamepad1.x) {
                            position = START.BLUE_BACKDROP;
                            telemetry.clear();
                            telemetry.addLine("Selected Blue Backdrop");
                            telemetry.update();
                            break;
                        } else if (gamepad1.y) {
                            position = START.BLUE_STACK;
                            telemetry.clear();
                            telemetry.addLine("Selected Blue Stack");
                            telemetry.update();
                            break;
                        }
                    }
                    break;
                } else if (gamepad1.y) {
                    telemetry.clear();
                    telemetry.addLine("Press x for backstage side, y for stack side");
                    telemetry.update();
                    while (!gamepad1.x || !gamepad1.y) {
                        if (gamepad1.x) {
                            position = START.RED_BACKDROP;
                            telemetry.clear();
                            telemetry.addLine("Selected Red Backdrop");
                            telemetry.update();
                            break;
                        } else if (gamepad1.y) {
                            position = START.RED_STACK;
                            telemetry.clear();
                            telemetry.addLine("Selected Red Stack");
                            telemetry.update();
                            break;
                        }
                    }
                    break;
                }
            }
            telemetry.addLine("Press x to confirm, press y to restart");
            telemetry.update();
            while (!gamepad1.x || !gamepad1.y) {
                if(gamepad1.x){
                    exit = true;
                }
                else if(gamepad1.y){
                    exit = false;
                }
            }
            if(exit){
                break;
            }
        }
        mc = new MecanumBotConstant();
        drive = new SampleMecanumDrive(hardwareMap);

        telemetry.setMsTransmissionInterval(50);

        boxDetection = new BoxDetection(telemetry);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, mc.camera), cameraMonitorViewId);

        camera.setPipeline(boxDetection);
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
        Pose2d robotStart;
        switch(position){
            case BLUE_BACKDROP:
                robotStart = new Pose2d(15, 63, Math.toRadians(90));
                drive.setPoseEstimate(robotStart);
                //y = 29 for the right most april tag
                right = drive.trajectorySequenceBuilder(robotStart)
                        .lineTo(new Vector2d(15, 57))
                        .splineTo(new Vector2d(6,36), Math.toRadians(180))
                        .lineTo(new Vector2d(10, 36))
                        .splineToSplineHeading(new Pose2d(49, 29, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                //y = 35 for the middle april tag
                middle = drive.trajectorySequenceBuilder(robotStart)
                        .lineTo(new Vector2d(13, 30))
                        .splineToConstantHeading(new Vector2d(16, 34), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(49, 35, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                //y = 41 for the left april tag
                left =  drive.trajectorySequenceBuilder(robotStart)
                        .lineTo(new Vector2d(28, 40))
                        .lineToLinearHeading(new Pose2d(25, 45, Math.toRadians(270)))
                        .splineToSplineHeading(new Pose2d(49, 41, Math.toRadians(180)), Math.toRadians(0))
                        .build();
            case BLUE_STACK:
                robotStart = new Pose2d(-37.5, 63, Math.toRadians(90));
                drive.setPoseEstimate(robotStart);


                middle = drive.trajectorySequenceBuilder(robotStart)
                        .lineToLinearHeading(new Pose2d(-33.5, 31, Math.toRadians(90)))
                        .splineToLinearHeading(new Pose2d(-39,38, Math.toRadians(90)), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-54, 16, Math.toRadians(180)), Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(-24, 10, Math.toRadians(180)), Math.toRadians(0))
                        .lineTo(new Vector2d(20,10))
                        .splineToConstantHeading(new Vector2d(49,35), Math.toRadians(0))
                        .build();
                right = drive.trajectorySequenceBuilder(robotStart)
                        .lineToLinearHeading(new Pose2d(-38.5, 35, Math.toRadians(0)))
                        .lineTo(new Vector2d(-36.5, 35))
                        .splineToLinearHeading(new Pose2d(-34.5, 30.5, Math.toRadians(0)), Math.toRadians(270))
                        .lineTo(new Vector2d(-34.5, 26))
                        .splineToSplineHeading(new Pose2d(25,10, Math.toRadians(180)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(49,29), Math.toRadians(0))
                        .build();
                left =  drive.trajectorySequenceBuilder(robotStart)
                        .lineTo(new Vector2d(-37.5, 57))
                        .splineToSplineHeading(new Pose2d(-31,36, Math.toRadians(180)), Math.toRadians(0))
                        .lineTo(new Vector2d(-33, 36))
                        .splineToConstantHeading(new Vector2d(-25, 8), Math.toRadians(0))
                        .lineToSplineHeading(new Pose2d(25,8, Math.toRadians(180)))
                        .splineToConstantHeading(new Vector2d(49,41),Math.toRadians(0))
                        .build();
            case RED_BACKDROP:
                robotStart = new Pose2d(15, -63, Math.toRadians(270));

                right = drive.trajectorySequenceBuilder(robotStart)
                        .lineTo(new Vector2d(15, -57))
                        .splineTo(new Vector2d(6,-36), Math.toRadians(180))
                        .lineTo(new Vector2d(10, -36))
                        .splineToSplineHeading(new Pose2d(49, -29, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                middle = drive.trajectorySequenceBuilder(robotStart)
                        .lineTo(new Vector2d(13, -30))
                        .splineToConstantHeading(new Vector2d(16, -34), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(49, -35, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                //y = 41 for the left april tag
                left =  drive.trajectorySequenceBuilder(robotStart)
                        .lineTo(new Vector2d(28, -40))
                        .lineToLinearHeading(new Pose2d(25, -45, Math.toRadians(270)))
                        .splineToSplineHeading(new Pose2d(49, -41, Math.toRadians(180)), Math.toRadians(0))
                        .build();
            case RED_STACK:
                robotStart = new Pose2d(-37.5, -63, Math.toRadians(270));
                drive.setPoseEstimate(robotStart);


                middle = drive.trajectorySequenceBuilder(robotStart)
                        .lineToLinearHeading(new Pose2d(-33.50, -31.00, Math.toRadians(270.00)))
                        .splineToLinearHeading(new Pose2d(-39.00, -38.00, Math.toRadians(270.00)), Math.toRadians(180.00))
                        .splineToSplineHeading(new Pose2d(-54.00, -16.00, Math.toRadians(180.00)), Math.toRadians(90.00))
                        .splineToLinearHeading(new Pose2d(-24.00, -10.00, Math.toRadians(180.00)), Math.toRadians(360.00))
                        .lineTo(new Vector2d(20.00, -10.00))
                        .splineToConstantHeading(new Vector2d(49.00, -35.00), Math.toRadians(360.00))
                        .build();
                right = drive.trajectorySequenceBuilder(robotStart)
                        .lineToLinearHeading(new Pose2d(-38.50, -35.00, Math.toRadians(360.00)))
                        .lineTo(new Vector2d(-36.50, -35.00))
                        .splineToLinearHeading(new Pose2d(-34.50, -30.50, Math.toRadians(360.00)), Math.toRadians(90.00))
                        .lineTo(new Vector2d(-34.50, -26.00))
                        .splineToSplineHeading(new Pose2d(25.00, -10.00, Math.toRadians(180.00)), Math.toRadians(360.00))
                        .splineToConstantHeading(new Vector2d(49.00, -29.00), Math.toRadians(360.00))
                        .build();
                left =  drive.trajectorySequenceBuilder(robotStart)
                        .lineTo(new Vector2d(-37.50, -57.00))
                        .splineToSplineHeading(new Pose2d(-31.00, -36.00, Math.toRadians(180.00)), Math.toRadians(360.00))
                        .lineTo(new Vector2d(-33.00, -36.00))
                        .splineToConstantHeading(new Vector2d(-25.00, -8.00), Math.toRadians(360.00))
                        .lineToSplineHeading(new Pose2d(25.00, -8.00, Math.toRadians(180.00)))
                        .splineToConstantHeading(new Vector2d(49.00, -41.00), Math.toRadians(360.00))
                        .build();
        }



        telemetry.addData("We do a little trajectorys"," yes?");
        telemetry.update();
        while (!isStarted()) {
            telemetry.addData("Location", boxDetection.getLocation());
            telemetry.update();
            sleep(20);
            if(isStopRequested()){
                return;
            }

        }
//        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            break;
        }
    }
}
