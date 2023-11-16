package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.pipelines.BoxDetection;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subclasses.Lift;
import org.firstinspires.ftc.teamcode.subclasses.PlaneLauncher;
import org.firstinspires.ftc.teamcode.subclasses.VihasIntake;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous
public class MecanumRoadrunner extends LinearOpMode {
    MecanumBotConstant mc;
    OpenCvCamera camera;
    BoxDetection boxDetection;
    MecanumDrive drive;
    BoxDetection.Location location;
    TrajectorySequence middle, left, right;
    enum START{
        RED_BACKDROP,
        RED_STACK,
        BLUE_BACKDROP,
        BLUE_STACK
    }
    private boolean red = false;
    START position;
    public static Rect MIDDLE_TARGET = new Rect(
            new Point(150, 300),
            new Point(420, 540));
    public static Rect RIGHT_TARGET = new Rect(
            new Point(700, 300),
            new Point(960, 540));

    PlaneLauncher plane;
    VihasIntake intake;
    Lift lift;
    private boolean exit = false;
    @Override
    public void runOpMode(){
        plane = new PlaneLauncher(hardwareMap);
        intake = new VihasIntake(hardwareMap);
        lift = new Lift(hardwareMap);

        plane.moveTo(.1);
        intake.open();
//        lift.reset();

        location = BoxDetection.Location.LEFT;
        telemetry.addLine("Press x for Blue, y for red");
        telemetry.update();
        while (!isStopRequested()) {
            while (!isStopRequested()) {
                sleep(20);
                if (gamepad1.x) {
                    telemetry.clear();
                    telemetry.addLine("BLUE - Press x for backstage side, y for stack side");
                    telemetry.update();
                    red = false;
                    sleep(500);
                    while (!isStopRequested()) {
                        sleep(20);
                        if (gamepad1.x) {
                            position = START.BLUE_BACKDROP;
                            telemetry.clear();
                            telemetry.addLine("Selected Blue Backdrop");
                            telemetry.update();
                            sleep(100);
                            break;
                        }
                        if (gamepad1.y) {
                            position = START.BLUE_STACK;
                            telemetry.clear();
                            telemetry.addLine("Selected Blue Stack");
                            telemetry.update();
                            sleep(100);
                            break;
                        }
                    }
                    break;
                }
                if (gamepad1.y) {
                    telemetry.clear();
                    telemetry.addLine("RED - Press x for backstage side, y for stack side");
                    telemetry.update();
                    red = true;
                    sleep(500);
                    while (!isStopRequested()) {
                        if (gamepad1.x) {
                            position = START.RED_BACKDROP;
                            telemetry.clear();
                            telemetry.addLine("Selected Red Backdrop");
                            telemetry.update();
                            sleep(100);
                            break;
                        }
                        if (gamepad1.y) {
                            position = START.RED_STACK;
                            telemetry.clear();
                            telemetry.addLine("Selected Red Stack");
                            telemetry.update();
                            sleep(100);
                            break;
                        }
                        sleep(20);

                    }
                    break;
                }
            }

            telemetry.addLine("Press x to confirm, press y to restart");
            telemetry.update();
            while (!isStopRequested()) {
                if (gamepad1.x) {
                    exit = true;
                    break;
                }
                if (gamepad1.y) {
                    exit = false;
                    break;
                }
                sleep(20);
            }
            if(exit){
                break;
            }
        }

        mc = new MecanumBotConstant();
        drive = new MecanumDrive(hardwareMap);

        telemetry.setMsTransmissionInterval(50);
        switch (position){
            //(60,404), (230, 544) BLUE STACK and RED BACKDROP
            //(220,404), (440, 544) BLUE BACKDROP AND RED STACK

            //(450,404), (640, 544) BLUE STACK and RED BACKDROP
            //(640,404), (900, 544) BLUE BACKDROP AND RED STACK
            case BLUE_BACKDROP:
            case RED_STACK:
                MIDDLE_TARGET = new Rect(
                    new Point(220, 404),
                    new Point(440, 544));
                RIGHT_TARGET = new Rect(
                        new Point(640, 404),
                        new Point(900, 540));
                break;
            case BLUE_STACK:
            case RED_BACKDROP:
                MIDDLE_TARGET = new Rect(
                        new Point(60, 404),
                        new Point(230, 544));
                RIGHT_TARGET = new Rect(
                        new Point(450, 404),
                        new Point(640, 540));
                break;

        }
        boxDetection = new BoxDetection(telemetry, MIDDLE_TARGET, RIGHT_TARGET,red);
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
                        .splineToSplineHeading(new Pose2d(6,33, Math.toRadians(45)), Math.toRadians(180))
                        .lineTo(new Vector2d(10, 36))
                        .addTemporalMarker(.5, new MarkerCallback() {
                            @Override
                            public void onMarkerReached() {
                                plane.setFlat();
                            }
                        })
                        .splineToSplineHeading(new Pose2d(49, 29, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                //y = 35 for the middle april tag
                middle = drive.trajectorySequenceBuilder(robotStart)
                        .lineTo(new Vector2d(13, 30))
                        .lineTo(new Vector2d(13,31))
                        .addTemporalMarker(.5, new MarkerCallback() {
                            @Override
                            public void onMarkerReached() {
                                plane.setFlat();
                            }
                        })
                        .splineToConstantHeading(new Vector2d(16, 34), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(49, 35, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                //y = 41 for the left april tag
                left =  drive.trajectorySequenceBuilder(robotStart)
                        .lineTo(new Vector2d(28, 36))
                        .addTemporalMarker(.5, new MarkerCallback() {
                            @Override
                            public void onMarkerReached() {
                                plane.setFlat();
                            }
                        })
                        .lineToLinearHeading(new Pose2d(25, 45, Math.toRadians(90)))
                        .splineToSplineHeading(new Pose2d(49, 41, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                break;
            case BLUE_STACK:
                robotStart = new Pose2d(-37.5, 63, Math.toRadians(90));
                drive.setPoseEstimate(robotStart);


                middle = drive.trajectorySequenceBuilder(robotStart)
                        .lineToLinearHeading(new Pose2d(-33.5, 31, Math.toRadians(90)))
                        .lineTo(new Vector2d(-33.5, 32))
                        .addTemporalMarker(.5, new MarkerCallback() {
                            @Override
                            public void onMarkerReached() {
                                plane.setFlat();
                            }
                        })
                        .splineToLinearHeading(new Pose2d(-39,38, Math.toRadians(90)), Math.toRadians(180))
                        .splineToSplineHeading(new Pose2d(-54, 16, Math.toRadians(180)), Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(-24, 9.2, Math.toRadians(180)), Math.toRadians(0))
                        .lineTo(new Vector2d(20,10))
                        .splineToConstantHeading(new Vector2d(49,35), Math.toRadians(0))
                        .build();
                right = drive.trajectorySequenceBuilder(robotStart)
                        .lineToLinearHeading(new Pose2d(-38.5, 35, Math.toRadians(0)))
                        .lineTo(new Vector2d(-36.5, 35))
                        .addTemporalMarker(.5, new MarkerCallback() {
                            @Override
                            public void onMarkerReached() {
                                plane.setFlat();
                            }
                        })
                        .splineToLinearHeading(new Pose2d(-34.5, 30.5, Math.toRadians(0)), Math.toRadians(270))
                        .lineTo(new Vector2d(-34.5, 26))
                        .splineToSplineHeading(new Pose2d(25,9.2, Math.toRadians(180)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(49,29), Math.toRadians(0))
                        .build();
                left =  drive.trajectorySequenceBuilder(robotStart)
                        .lineTo(new Vector2d(-37.5, 57))
                        .addTemporalMarker(.5, new MarkerCallback() {
                            @Override
                            public void onMarkerReached() {
                                plane.setFlat();
                            }
                        })
                        .splineToSplineHeading(new Pose2d(-31,36, Math.toRadians(180)), Math.toRadians(0))
                        .lineTo(new Vector2d(-33, 36))
                        .splineToConstantHeading(new Vector2d(-25, 9.5), Math.toRadians(0))
                        .lineToSplineHeading(new Pose2d(25,9.2, Math.toRadians(180)))
                        .splineToConstantHeading(new Vector2d(49,41),Math.toRadians(0))
                        .build();
                break;
            case RED_BACKDROP:
                robotStart = new Pose2d(15, -63, Math.toRadians(270));

                right = drive.trajectorySequenceBuilder(robotStart)
                        .addTemporalMarker(.5, new MarkerCallback() {
                            @Override
                            public void onMarkerReached() {
                                plane.setFlat();
                            }
                        })
                        .lineTo(new Vector2d(15, -57))
                        .splineTo(new Vector2d(6,-34), Math.toRadians(180))
                        .lineTo(new Vector2d(10, -36))
                        .splineToSplineHeading(new Pose2d(49, -29, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                middle = drive.trajectorySequenceBuilder(robotStart)
                        .lineTo(new Vector2d(13, -30))
                        .addTemporalMarker(.5, new MarkerCallback() {
                            @Override
                            public void onMarkerReached() {
                                plane.setFlat();
                            }
                        })
                        .lineTo(new Vector2d(13,-31))
                        .splineToConstantHeading(new Vector2d(16, -34), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(49, -35, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                //y = 41 for the left april tag
                left =  drive.trajectorySequenceBuilder(robotStart)
                        .lineTo(new Vector2d(28, -36)).addTemporalMarker(.5, new MarkerCallback() {
                            @Override
                            public void onMarkerReached() {
                                plane.setFlat();
                            }
                        })

                        .lineToLinearHeading(new Pose2d(25, -45, Math.toRadians(270)))
                        .splineToSplineHeading(new Pose2d(49, -41, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                break;
            case RED_STACK:
                robotStart = new Pose2d(-37.5, -63, Math.toRadians(270));
                drive.setPoseEstimate(robotStart);


                middle = drive.trajectorySequenceBuilder(robotStart)
                        .lineToLinearHeading(new Pose2d(-33.50, -31.00, Math.toRadians(270.00)))
                        .addTemporalMarker(.5, new MarkerCallback() {
                            @Override
                            public void onMarkerReached() {
                                plane.setFlat();
                            }
                        })
                        .lineTo(new Vector2d(-33.5, -32))
                        .splineToLinearHeading(new Pose2d(-39.00, -38.00, Math.toRadians(270.00)), Math.toRadians(180.00))
                        .splineToSplineHeading(new Pose2d(-54.00, -16.00, Math.toRadians(180.00)), Math.toRadians(90.00))
                        .splineToLinearHeading(new Pose2d(-24.00, -10.00, Math.toRadians(180.00)), Math.toRadians(360.00))
                        .lineTo(new Vector2d(20.00, -10.00))
                        .splineToConstantHeading(new Vector2d(49.00, -35.00), Math.toRadians(360.00))
                        .build();
                right = drive.trajectorySequenceBuilder(robotStart)
                        .lineToLinearHeading(new Pose2d(-38.50, -35.00, Math.toRadians(360.00)))
                        .addTemporalMarker(.5, new MarkerCallback() {
                            @Override
                            public void onMarkerReached() {
                                plane.setFlat();
                            }
                        })
                        .lineTo(new Vector2d(-36.50, -35.00))
                        .splineToLinearHeading(new Pose2d(-34.50, -30.50, Math.toRadians(360.00)), Math.toRadians(90.00))
                        .lineTo(new Vector2d(-34.50, -26.00))
                        .splineToSplineHeading(new Pose2d(25.00, -10.00, Math.toRadians(180.00)), Math.toRadians(360.00))
                        .splineToConstantHeading(new Vector2d(49.00, -29.00), Math.toRadians(360.00))
                        .build();
                left =  drive.trajectorySequenceBuilder(robotStart)
                        .lineTo(new Vector2d(-37.50, -57.00))
                        .addTemporalMarker(.5, new MarkerCallback() {
                            @Override
                            public void onMarkerReached() {
                                plane.setFlat();
                            }
                        })
                        .splineToSplineHeading(new Pose2d(-31.00, -36.00, Math.toRadians(180.00)), Math.toRadians(360.00))
                        .lineTo(new Vector2d(-33.00, -36.00))
                        .splineToConstantHeading(new Vector2d(-25.00, -9.500), Math.toRadians(360.00))
                        .lineToSplineHeading(new Pose2d(25.00, -9.500, Math.toRadians(180.00)))
                        .splineToConstantHeading(new Vector2d(49.00, -41.00), Math.toRadians(360.00))
                        .build();
                break;
        }


        while (!isStarted()) {
            telemetry.addData("Location", boxDetection.getLocation());
            telemetry.update();
            location = boxDetection.getLocation();
            sleep(20);
            if(isStopRequested()){
                return;
            }

        }
        switch (location){
            case LEFT:
                drive.followTrajectorySequenceAsync(left);
                break;
            case RIGHT:
                drive.followTrajectorySequenceAsync(right);
                break;
            case MIDDLE:
                drive.followTrajectorySequenceAsync(middle);
                break;
        }
        while(!isStopRequested() && opModeIsActive()){
            drive.update();
        }
    }
}
