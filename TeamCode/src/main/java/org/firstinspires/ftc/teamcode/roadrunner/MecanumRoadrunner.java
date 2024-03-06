package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DataTransfer;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.pipelines.BoxDetection;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subclasses.Color;
import org.firstinspires.ftc.teamcode.subclasses.Distance;
import org.firstinspires.ftc.teamcode.subclasses.Intake;
import org.firstinspires.ftc.teamcode.subclasses.PlaneLauncher;
import org.firstinspires.ftc.teamcode.subclasses.ShivaniRigging;
import org.firstinspires.ftc.teamcode.subclasses.Unicorn;
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
    TrajectorySequence middle, left, right, intermediate, score;
    TrajectorySequence intoBackdrop;

    private double strafe_timer;

    ElapsedTime timer;

    enum START{
        RED_BACKDROP,
        RED_STACK,
        BLUE_BACKDROP,
        BLUE_STACK
    }

    private enum AUTO_STATES{
        FIRST_PATH,
        IDENTIFY_SPOT
    }
    private AUTO_STATES auto_states;
    private Vector2d left_blue = new Vector2d(49,40);
    private Vector2d middle_blue = new Vector2d(49,34);
    private Vector2d right_blue = new Vector2d(49, 30.5);

    private Vector2d left_red = new Vector2d(49, -31);
    private Vector2d middle_red = new Vector2d(49,-37);
    private Vector2d right_red = new Vector2d(49,-42);

    private boolean red = false;
    private boolean stack = false;
    private boolean crashed = false;

    private boolean middle_to_middle_of_field = false;

    START position;
    public static Rect MIDDLE_TARGET = new Rect(
            new Point(150, 300),
            new Point(420, 540));
    public static Rect OTHER_TARGET = new Rect(
            new Point(700, 300),
            new Point(960, 540));

    PlaneLauncher plane;

//    Lift lift;
    Unicorn unicorn;
    Distance distance;
    Intake intake;
    ShivaniRigging rigging;

    Color color;
    private float[] colors;

    private boolean exit = false;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        DataTransfer.init = false;
        auto_states = AUTO_STATES.FIRST_PATH;

        plane = new PlaneLauncher(hardwareMap);
        unicorn = new Unicorn(hardwareMap, telemetry);
        distance = new Distance(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        rigging = new ShivaniRigging(hardwareMap, telemetry);
//
        intake.init();
        plane.init();
        unicorn.init();
        rigging.init();
        distance.init();





        location = BoxDetection.Location.LEFT;
        telemetry.clear();
        telemetry.addLine("x for Blue Backstage");
        telemetry.addLine("a for Blue Stack");
        telemetry.addLine("y for Red Backstage");
        telemetry.addLine("b for Red Stack");
        telemetry.update();
        
        while (!isStopRequested()) {
            while (!isStopRequested()) {
                sleep(20);
                if (gamepad1.x) {
                    telemetry.clear();
                    telemetry.addLine("Selected Blue Backstage");
                    red = false;
                    stack = false;
                    position = START.BLUE_BACKDROP;
                    break;
                }
                if (gamepad1.y) {
                    telemetry.clear();
                    telemetry.addLine("Selected Red Backstage");
                    telemetry.update();
                    stack = false;
                    red = true;
                    position = START.RED_BACKDROP;

                    break;
                }if (gamepad1.a) {
                    telemetry.clear();
                    telemetry.addLine("Selected Blue Stack");
                    position = START.BLUE_STACK;
                    stack = true;
                    red = false;

                    break;
                }
                if (gamepad1.b) {
                    telemetry.clear();
                    telemetry.addLine("Selected Red Stack");
                    telemetry.update();
                    position = START.RED_STACK;
                    stack = true;
                    red = true;
                    break;
                }
            }
            sleep(200);
            telemetry.addLine("Press x to confirm, press y to restart");
            telemetry.update();
            while (!isStopRequested()) {
                if (gamepad1.x) {
                    exit = true;
                    break;
                }
                if (gamepad1.y) {
                    exit = false;
                    sleep(1000);
                    break;
                }
                sleep(20);
            }
            if(exit){
                break;
            }
        }
        DataTransfer.red = red;

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
                        new Point(440, 250),
                        new Point(580, 406)
                );

                OTHER_TARGET = new Rect(
                        new Point(50, 250),
                        new Point(160, 406)
                );
                break;
            case BLUE_STACK:
            case RED_BACKDROP:
                MIDDLE_TARGET = new Rect(
                        new Point(670, 250),
                        new Point(800, 406)
                );

                OTHER_TARGET = new Rect(


                        new Point(230, 230),
                        new Point(370, 380)
                );
                break;

        }

        boxDetection = new BoxDetection(telemetry, MIDDLE_TARGET, OTHER_TARGET, red, !stack);
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






        switch(position) {
            case BLUE_BACKDROP:
                robotStart = new Pose2d(16.5, 63, Math.toRadians(90));
                drive.setPoseEstimate(robotStart);
                //y = 29 for the right most april tag
                right = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(8.00, 35.5, Math.toRadians(0)), Math.toRadians(180))
                        .setReversed(false)
                        .forward(3)
                        .splineTo(new Vector2d(20.50, 40.5), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(25.50, 45.5), Math.toRadians(0))

                        .splineToConstantHeading(new Vector2d(32.50, 40.5), Math.toRadians(270))
                        .splineTo(right_blue, Math.toRadians(0))
                        .build();
                //y = 35 for the middle april tag
                middle = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(15.50, 30.5, Math.toRadians(90)), Math.toRadians(270))
                        .forward(3)
                        .splineToConstantHeading(new Vector2d(25.50, 48.5), Math.toRadians(0))

                        .splineToConstantHeading(new Vector2d(32.50, 44.5), Math.toRadians(270))

                        .splineTo(middle_blue, Math.toRadians(0))
                        .build();
                //y = 41 for the left april tag
                left = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(22.50, 36.5, Math.toRadians(90)), Math.toRadians(270))
                        .forward(3)
                        .splineToConstantHeading(new Vector2d(25.50, 54.5), Math.toRadians(0))

                        .splineToConstantHeading(new Vector2d(32.50, 48.5), Math.toRadians(270))

                        .splineTo(left_blue, Math.toRadians(0))

                        .build();
                break;
            case RED_BACKDROP:
                robotStart = new Pose2d(16.5, -63, Math.toRadians(270));
                drive.setPoseEstimate(robotStart);
                right = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(25.50, -36.5, Math.toRadians(270)), Math.toRadians(90))
                        .forward(3)
                        .splineToConstantHeading(new Vector2d(25.50, -48.5), Math.toRadians(0))

                        .splineToConstantHeading(new Vector2d(32.50, -44.5), Math.toRadians(90))

                        .splineTo(right_red, Math.toRadians(0))
                        .build();
                middle = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(16.50, -30.5, Math.toRadians(270)), Math.toRadians(90))
                        .forward(3)
                        .splineToConstantHeading(new Vector2d(25.50, -48.5), Math.toRadians(0))

                        .splineToConstantHeading(new Vector2d(32.50, -44.5), Math.toRadians(90))

                        .splineTo(middle_red, Math.toRadians(0))

                        .build();
                left = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(8.00, -33.5, Math.toRadians(315)), Math.toRadians(180))
                        .setReversed(false)
                        .forward(3)
                        .splineTo(new Vector2d(20.50, -40.5), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(25.50, -45.5), Math.toRadians(0))

                        .splineToConstantHeading(new Vector2d(32.50, -40.5), Math.toRadians(90))
                        .splineTo(left_red, Math.toRadians(0))

                        .build();
                break;


            case BLUE_STACK:
                robotStart = new Pose2d(-37.5, 63, Math.toRadians(90));
                drive.setPoseEstimate(robotStart);

                middle = drive.trajectorySequenceBuilder(robotStart)//updated
                        .lineToLinearHeading(new Pose2d(-34.00, 30.5, Math.toRadians(90)))
                        .lineTo(new Vector2d(-34, 33))
                        .splineTo(new Vector2d(-39,45), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-44,50), Math.toRadians(90))
                        .strafeTo(new Vector2d(-44,54))
                        .splineToConstantHeading(new Vector2d(-36,61), Math.toRadians(0))
                        .back(64)
                        .splineToConstantHeading(middle_blue, Math.toRadians(0))

                        .build();
                right = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-46.00, 32.5, Math.toRadians(90)), Math.toRadians(270))
                        .setReversed(false)
                        .splineTo(new Vector2d(-47,50), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-50,61, Math.toRadians(180)), Math.toRadians(0))

                        .back(60)

                        .splineToConstantHeading(right_blue, Math.toRadians(0))
                        .build();
                left = drive.trajectorySequenceBuilder(robotStart)

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-29.00, 35.5, Math.toRadians(135)), Math.toRadians(359))
                        .setReversed(false)
                        .forward(12)
                        .splineTo(new Vector2d(-45,45), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-48,50), Math.toRadians(90))
                        .strafeTo(new Vector2d(-48,54))
                        .splineToConstantHeading(new Vector2d(-36,61), Math.toRadians(0))
                        .back(64)
                        .splineToConstantHeading(left_blue, Math.toRadians(0))
                        .build();
                break;
            case RED_STACK:
                robotStart = new Pose2d(-37.5, -63, Math.toRadians(270));
                drive.setPoseEstimate(robotStart);
                //y = 41 for the left april tag
                right = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-26.50, -35.5, Math.toRadians(200)), Math.toRadians(359))
                        .setReversed(false)
                        .forward(12)
                        .splineTo(new Vector2d(-45,-45), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-48,-50), Math.toRadians(270))
                        .strafeTo(new Vector2d(-48,-54))
                        .splineToConstantHeading(new Vector2d(-36,-59), Math.toRadians(0))
                        .back(64)
                        .splineToConstantHeading(right_red, Math.toRadians(0))
                        .build();
                //y = 29 for the right most april tag
                left = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-42.00, -35.5, Math.toRadians(270)), Math.toRadians(90))
                        .setReversed(false)

                        .splineTo(new Vector2d(-47,-50), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-50,-59, Math.toRadians(180)), Math.toRadians(0))

                        .back(66)

                        .splineToConstantHeading(left_red, Math.toRadians(0))
                        .build();
                //y = 35 for the middle april tag
                middle = drive.trajectorySequenceBuilder(robotStart)
                        .lineToLinearHeading(new Pose2d(-34.00, -29.5, Math.toRadians(270)))
                        .lineTo(new Vector2d(-34, -33))
                        .splineTo(new Vector2d(-39,-45), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-44,-50), Math.toRadians(270))
                        .strafeTo(new Vector2d(-44,-54))
                        .splineToConstantHeading(new Vector2d(-36,-59), Math.toRadians(0))
                        .back(64)
                        .splineToConstantHeading(middle_red, Math.toRadians(0))
                        .build();
                break;
        }

        timer = new ElapsedTime();
        while (!isStarted()) {
            telemetry.addData("Location", boxDetection.getLocation());
            telemetry.update();
            location = boxDetection.getLocation();
            sleep(20);
            if(isStopRequested()){
                return;
            }

        }

        unicorn.travel();
        if(stack){
            sleep(10000);
        }
        rigging.release_hooks();
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
        double strafe_power = location.equals(BoxDetection.Location.RIGHT) ? 0.4 : -0.4;

        timer.reset();
        while(!isStopRequested() && opModeIsActive()){
            drive.update();
            rigging.update();
            if(Math.abs(drive.getLastError().getX()) > 12 || Math.abs(drive.getLastError().getY()) > 12){

                drive.breakFollowing();
                drive.setDrivePower(new Pose2d());
                crashed = true;
                if(drive.getExternalHeading() > Math.toRadians(173) && drive.getExternalHeading() < Math.toRadians(187)){ //Basically are we within 7 degrees of 180 since hitting the truss is a different error or something.
                    telemetry.addLine("Collide with delbotics?");
                    telemetry.update();
                }
                DataTransfer.delivered = false;

                return;


            }
            switch(auto_states){
                case FIRST_PATH:
                    if (!drive.isBusy()) {
                        if(!crashed) {
                            if (!drive.isBusy()) {
                                rigging.setHookPower(0);
                                sleep(2000);
                                Pose2d startPose = drive.getPoseEstimate();
                                intoBackdrop = drive.trajectorySequenceBuilder(startPose)
                                        .lineToLinearHeading(new Pose2d(startPose.getX() + getAdjustedDistance() + 1, startPose.getY(), Math.toRadians(180)),
                                                drive.getVelocityConstraint(DriveConstants.MAX_VEL * .10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)

                                        )
                                        .build();
                                drive.followTrajectorySequence(intoBackdrop);
                                strafe_timer = timer.time();
                                colors = color.getRGBValues();
                                if(colors[0] < 0.014){
                                    deliver();
                                    return;
                                }
                                drive.setWeightedDrivePower(new Pose2d(0, strafe_power));

                                auto_states = AUTO_STATES.IDENTIFY_SPOT;
                            }
                        }
                    }
                case IDENTIFY_SPOT:
                    colors = color.getRGBValues();
                    telemetry.addData("red", colors[0]);
                    telemetry.addData("green", colors[1]);
                    telemetry.addData("blue", colors[2]);
                    if(colors[0] < 0.014 || timer.seconds() - strafe_timer > 3){
                        drive.setWeightedDrivePower(new Pose2d());
                        if(strafe_power > 0) {
                            intoBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .strafeLeft(2)
                                    .back(1,
                                            drive.getVelocityConstraint(DriveConstants.MAX_VEL * .10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();
                        }
                        else{
                            intoBackdrop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .back(1,
                                            drive.getVelocityConstraint(DriveConstants.MAX_VEL * .10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();
                        }
                        drive.followTrajectorySequence(intoBackdrop);
                        deliver();
                        park();
                        return;
                    }



            }


        }
    }
    public double getAdjustedDistance(){
        double output = distance.getDistFromRobotEdge()+ 0.05;
        if (output > 78){
            output = 8;
        }
        return output;

    }
    public void park(){
        if (!stack) {

            Vector2d endPose = new Vector2d();
            if (red) {
                if(location == BoxDetection.Location.LEFT || (location == BoxDetection.Location.MIDDLE && !middle_to_middle_of_field)) {
                    endPose = new Vector2d(50, -61);
                }else{
                    endPose = new Vector2d(50, -11);
                }
            } else {
                if(location == BoxDetection.Location.LEFT || (location == BoxDetection.Location.MIDDLE && middle_to_middle_of_field)) {
                    endPose = new Vector2d(50,11);

                }else{
                    endPose = new Vector2d(50, 64);

                }
            }
            TrajectorySequence intoPark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .forward(6)
                    .addDisplacementMarker(() -> {
                        unicorn.stow();
                    })
                    .waitSeconds(0.3)
                    .splineToConstantHeading(endPose, Math.toRadians(0))

                    .build();
            drive.followTrajectorySequence(intoPark);
            Trajectory last = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .back(getAdjustedDistance() - 2,
                            drive.getVelocityConstraint(DriveConstants.MAX_VEL * .60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            drive.followTrajectory(last);

        }else{
            unicorn.stow();
        }
    }
    public void deliver(){
//        sleep(500);
        unicorn.deliver();
        sleep(1400);
        unicorn.goToPosition(0.45);
        sleep(500);
        unicorn.deliver();
        sleep(500);
    }



}
