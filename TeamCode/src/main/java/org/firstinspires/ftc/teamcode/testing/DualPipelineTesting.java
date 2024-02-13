package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.pipelines.BoxDetection.Location.LEFT;
import static org.firstinspires.ftc.teamcode.pipelines.BoxDetection.Location.MIDDLE;
import static org.firstinspires.ftc.teamcode.pipelines.BoxDetection.Location.RIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.IMUTransfer;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.pipelines.BoxDetection;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subclasses.Distance;
import org.firstinspires.ftc.teamcode.subclasses.DualVisionProcessor;
import org.firstinspires.ftc.teamcode.subclasses.Intake;
import org.firstinspires.ftc.teamcode.subclasses.Lift;
import org.firstinspires.ftc.teamcode.subclasses.PlaneLauncher;
import org.firstinspires.ftc.teamcode.subclasses.ShivaniRigging;
import org.firstinspires.ftc.teamcode.subclasses.Unicorn;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.concurrent.TimeUnit;


@Config
@TeleOp
public class DualPipelineTesting extends LinearOpMode {
    MecanumBotConstant mc;
    OpenCvCamera camera;
    DualVisionProcessor dualVisionProcessor;
    BoxDetection boxDetection;
    MecanumDrive drive;
    BoxDetection.Location location;
    TrajectorySequence middle, left, right, intermediate, score;
    TrajectorySequence intoBackdrop;
    enum START{
        RED_BACKDROP,
        RED_STACK,
        BLUE_BACKDROP,
        BLUE_STACK
    }
    private final double BLUE_SIDE_RIGHT = 28.5;
    private final double BLUE_SIDE_MIDDLE = 32;
    private final double BLUE_SIDE_LEFT = 38;
    public static boolean attemptApril = false;
    private final double RED_SIDE_LEFT = -28.5;
    private final double RED_SIDE_MIDDLE = -32;//c
    private final double RED_SIDE_RIGHT = -38;


    private Vector2d left_blue = new Vector2d(49,38);
    private Vector2d middle_blue = new Vector2d(49,32);
    private Vector2d right_blue = new Vector2d(49, 26);

    private Vector2d left_red = new Vector2d(49, -26);
    private Vector2d middle_red = new Vector2d(49,-32);
    private Vector2d right_red = new Vector2d(49,-38);

    private boolean red = false;
    private boolean stack = false;

    START position;
    public static Rect MIDDLE_TARGET = new Rect(
            new Point(150, 300),
            new Point(420, 540));
    public static Rect OTHER_TARGET = new Rect(
            new Point(700, 300),
            new Point(960, 540));
    public static int target_id = 2;
    PlaneLauncher plane;

    //    VihasIntake intake;
    Lift lift;
    Unicorn delivery;
    Distance distance;
    Intake intake;
    ShivaniRigging rigging;
    private boolean exit = false;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        location = LEFT;

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
                    sleep(200);
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
                        new Point(520, 356),
                        new Point(680, 500)
                );

                OTHER_TARGET = new Rect(
                        new Point(120, 356),
                        new Point(300, 520)
                );
                break;
            case BLUE_STACK:
            case RED_BACKDROP:
                MIDDLE_TARGET = new Rect(
                        new Point(360, 356),
                        new Point(520, 500)
                );

                OTHER_TARGET = new Rect(
                        new Point(780, 356),
                        new Point(960, 520)
                );
                break;

        }
        boxDetection= new BoxDetection(telemetry, MIDDLE_TARGET, OTHER_TARGET, red, !stack);
        dualVisionProcessor = new DualVisionProcessor(hardwareMap, boxDetection);




        Pose2d robotStart;

        switch(position) {
            case BLUE_BACKDROP:
                robotStart = new Pose2d(16.5, 63, Math.toRadians(90));
                drive.setPoseEstimate(robotStart);
                //y = 29 for the right most april tag
                right = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(8.50, 34.5, Math.toRadians(45)), Math.toRadians(270))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(right_blue, Math.toRadians(180)), Math.toRadians(0))
                        .build();
                //y = 35 for the middle april tag
                middle = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(16.50, 30.5, Math.toRadians(90)), Math.toRadians(270))
                        .forward(3)
                        .splineToConstantHeading(new Vector2d(25.50, 48.5), Math.toRadians(0))

                        .splineToConstantHeading(new Vector2d(32.50, 44.5), Math.toRadians(270))

                        .splineTo(middle_blue, Math.toRadians(0))
                        .build();
                //y = 41 for the left april tag
                left = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(25.50, 36.5, Math.toRadians(90)), Math.toRadians(270))
                        .forward(3)
                        .splineToConstantHeading(new Vector2d(25.50, 60.5), Math.toRadians(0))

                        .splineToConstantHeading(new Vector2d(32.50, 56.5), Math.toRadians(270))

                        .splineTo(left_blue, Math.toRadians(0))

                        .build();
                break;

            case BLUE_STACK:
                robotStart = new Pose2d(-37.5, 63, Math.toRadians(90));
                drive.setPoseEstimate(robotStart);
                middle = drive.trajectorySequenceBuilder(robotStart)
                        .lineToLinearHeading(new Pose2d(-34.00, 29.5, Math.toRadians(90)))
                        .lineTo(new Vector2d(-34, 33))
                        .splineTo(new Vector2d(-45,45), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-52,24), Math.toRadians(270))
                        .strafeTo(new Vector2d(-52,20))
                        .splineToConstantHeading(new Vector2d(-36,12), Math.toRadians(0))
                        .back(64)
                        .splineToConstantHeading(middle_blue, Math.toRadians(0))

                        .build();
                right = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-27.00, 35.5, Math.toRadians(135)), Math.toRadians(359))
                        .setReversed(false)
                        .forward(12)
                        .splineTo(new Vector2d(-45,45), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-52,24), Math.toRadians(270))
                        .strafeTo(new Vector2d(-52,20))
                        .splineToConstantHeading(new Vector2d(-36,12), Math.toRadians(0))
                        .back(64)
                        .splineToConstantHeading(right_blue, Math.toRadians(0))

                        .build();
                left = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-42.00, 32.5, Math.toRadians(90)), Math.toRadians(270))
                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(-42,45), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-35, 42), Math.toRadians(270))
                        .splineTo(new Vector2d(-28,12), Math.toRadians(0))
                        .back(66)

                        .splineToConstantHeading(left_blue, Math.toRadians(0))

                        .build();
                break;


            case RED_BACKDROP:
                robotStart = new Pose2d(-37.5, 63, Math.toRadians(90));
                drive.setPoseEstimate(robotStart);
                middle = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(16.50, -30.5, Math.toRadians(270)), Math.toRadians(90))
                        .forward(3)
                        .splineToConstantHeading(new Vector2d(25.50, -48.5), Math.toRadians(0))

                        .splineToConstantHeading(new Vector2d(32.50, -44.5), Math.toRadians(90))

                        .splineTo(middle_red, Math.toRadians(0))

                        .build();
                right = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(25.50, -36.5, Math.toRadians(270)), Math.toRadians(90))
                        .forward(3)
                        .splineToConstantHeading(new Vector2d(25.50, -60.5), Math.toRadians(0))

                        .splineToConstantHeading(new Vector2d(32.50, -56.5), Math.toRadians(90))

                        .splineTo(right_red, Math.toRadians(0))
                        .build();
                left = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(8.50, -34.5, Math.toRadians(315)), Math.toRadians(90))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(left_red, Math.toRadians(180)), Math.toRadians(0))

                        .build();
                break;
            case RED_STACK:
                robotStart = new Pose2d(16.5, 63, Math.toRadians(90));
                drive.setPoseEstimate(robotStart);
                //y = 29 for the right most april tag
                right = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-42.00, -32.5, Math.toRadians(270)), Math.toRadians(90))
                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(-42,-45), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-35, -42), Math.toRadians(90))
                        .splineTo(new Vector2d(-28,-12), Math.toRadians(0))
                        .back(66)

                        .splineToConstantHeading(right_red, Math.toRadians(0))
                        .build();
                //y = 35 for the middle april tag
                middle = drive.trajectorySequenceBuilder(robotStart)
                        .lineToLinearHeading(new Pose2d(-34.00, -29.5, Math.toRadians(270)))
                        .lineTo(new Vector2d(-34, -33))
                        .splineTo(new Vector2d(-45,-45), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-52,-24), Math.toRadians(90))
                        .strafeTo(new Vector2d(-52,-20))
                        .splineToConstantHeading(new Vector2d(-36,-12), Math.toRadians(0))
                        .back(64)
                        .splineToConstantHeading(middle_red, Math.toRadians(0))
                        .build();
                //y = 41 for the left april tag
                left = drive.trajectorySequenceBuilder(robotStart)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-27.00, -35.5, Math.toRadians(225)), Math.toRadians(359))
                        .setReversed(false)
                        .forward(12)
                        .splineTo(new Vector2d(-45,-45), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(-52,-24), Math.toRadians(90))
                        .strafeTo(new Vector2d(-52,-20))
                        .splineToConstantHeading(new Vector2d(-36,-12), Math.toRadians(0))
                        .back(64)
                        .splineToConstantHeading(left_red, Math.toRadians(0))
                        .build();
                break;
        }

        ElapsedTime timer = new ElapsedTime();
        double delay = 500;
        while (!isStarted()) {
            telemetry.addData("Location", dualVisionProcessor.getLocation());
            telemetry.update();
            location = dualVisionProcessor.getLocation();
            sleep(20);
            if(isStopRequested()){
                return;
            }

        }


        timer.reset();
        dualVisionProcessor.activeAprilTag();

        while(!isStopRequested() && opModeIsActive()){
            drive.update();
            if(attemptApril){
                sleep(500);
                Pose2d startPose = drive.getPoseEstimate();
                attemptApril = false;
                intoBackdrop = getAdjustedPath(startPose);
                drive.followTrajectorySequence(intoBackdrop);

            }
        }
    }
    public TrajectorySequence getAdjustedPath(Pose2d robotPosition){
        int tagid = -1;
        switch (location){
            case LEFT:
                tagid = 1;
                break;
            case MIDDLE:
                tagid = 2;
                break;
            case RIGHT:
                tagid = 3;
                break;
        }
        tagid += (tagid != -1 && red) ? 3 : 0;
        Pose2d moves = dualVisionProcessor.getTargetPos(tagid, 20);
        double x = moves.getX() + robotPosition.getX();
        double y = moves.getY() + robotPosition.getY();
        double turn = moves.getHeading() + robotPosition.getHeading();
        TrajectorySequence output = drive.trajectorySequenceBuilder(robotPosition)
                .lineToLinearHeading(new Pose2d(x,y, turn))
                .build();
        return output;

    }


}
