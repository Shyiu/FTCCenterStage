package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.pipelines.BoxDetection.Location.LEFT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.pipelines.BoxDetection;
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

import java.util.ArrayList;


@Config
@TeleOp
public class AprilTagMoveTesting extends LinearOpMode {
    MecanumBotConstant mc;
    OpenCvCamera camera;
    AprilTagPipeline aprilTagPipeline;
    MecanumDrive drive;
    BoxDetection.Location location;
    AprilTagDetection detection;
    public static boolean attemptApril = false;

    

 
    public static Rect MIDDLE_TARGET = new Rect(
            new Point(150, 300),
            new Point(420, 540));
    public static Rect OTHER_TARGET = new Rect(
            new Point(700, 300),
            new Point(960, 540));
    public static int tagid = 2;
    public ArrayList<Integer> targets;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        targets = new ArrayList<>();
        aprilTagPipeline = new AprilTagPipeline(hardwareMap);
      
        mc = new MecanumBotConstant();
        drive = new MecanumDrive(hardwareMap);

        targets.add(1);
        targets.add(2);
        targets.add(3);
        targets.add(4);
        targets.add(5);
        targets.add(6);

        telemetry.setMsTransmissionInterval(50);
        Pose2d robotStart;

       waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            drive.update();
            if(attemptApril){
                sleep(500);
                Pose2d startPose = drive.getPoseEstimate();
                attemptApril = false;
                if (detection.id == tagid){
                    TrajectorySequence intoBackdrop = getAdjustedPath(startPose, detection);
                    drive.followTrajectorySequence(intoBackdrop);
                }


            }
            detection = aprilTagPipeline.getDetectionsForTargets(targets);
            if (detection == null){
                continue;
            }
            telemetry.addData("Target", "ID %d (%s)", detection.id, detection.metadata.name);
            telemetry.addData("Range", "%5.1f inches", detection.ftcPose.range);//35 in
            telemetry.addData("Bearing", "%3.0f degrees", detection.ftcPose.bearing);//12 degress
            telemetry.addData("Yaw", "%3.0f degrees", detection.ftcPose.yaw);
            telemetry.addData("X", detection.center.x);
            telemetry.addData("Y", detection.center.y);
            telemetry.update();
        }
    }
    public TrajectorySequence getAdjustedPath(Pose2d robotPosition, AprilTagDetection detection){
//        int tagid = -1;
//        switch (location){
//            case LEFT:
//                tagid = 1;
//                break;
//            case MIDDLE:
//                tagid = 2;
//                break;
//            case RIGHT:
//                tagid = 3;
//                break;
//        }

        Pose2d moves = aprilTagPipeline.getTargetPos(detection, 20);
        double x = moves.getX() + robotPosition.getX();
        double y = moves.getY() + robotPosition.getY();
        double turn = moves.getHeading() + robotPosition.getHeading();
        TrajectorySequence output = drive.trajectorySequenceBuilder(robotPosition)
                .lineToLinearHeading(new Pose2d(x,y, turn))
                .build();
        return output;

    }


}
