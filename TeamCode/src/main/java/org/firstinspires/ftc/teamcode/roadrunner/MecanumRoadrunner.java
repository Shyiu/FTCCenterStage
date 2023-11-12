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

    @Override
    public void runOpMode(){
        mc = new MecanumBotConstant();
        drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(10.23, -63.74, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(16.99, -31.45, Math.toRadians(0.00)), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(11.73, -36.89), Math.toRadians(0.00))
                .splineTo(new Vector2d(49.28, -44.59), Math.toRadians(0.00))
                .build();
        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(10.62, -70.89, Math.toRadians(90.00)))
                .lineTo(new Vector2d(15, 57))
                .splineTo(new Vector2d(6,36), Math.toRadians(180))
                .build();
        TrajectorySequence testing =  drive.trajectorySequenceBuilder(new Pose2d(15, 63.00, Math.toRadians(90.00)))
                .lineTo(new Vector2d(14.5, 57))
                .splineTo(new Vector2d(18,34), Math.toRadians(0))
                .build();

        drive.setPoseEstimate(testing.start());
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
//        boxDetection.getLocation();
//        drive.followTrajectorySequenceAsync(testing);
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Location", boxDetection.getLocation());
            telemetry.update();
        }

        while(!isStopRequested() && opModeIsActive()){
            drive.followTrajectorySequence(testing);
            break;
        }
    }
}
