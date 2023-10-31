package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@TeleOp(name = "PipeLineTesting")
public class PipelineTesting extends LinearOpMode {
    AprilTagPipeline pipeline;
    ArrayList<Integer> targets;

    @Override
    public void runOpMode() throws InterruptedException {
        pipeline = new AprilTagPipeline(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        targets =  new ArrayList<>(Arrays.asList(1));
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            AprilTagDetection detection = pipeline.getDetectionsForTargets(targets);
            if (detection != null) {
                telemetry.addData("Target", "ID %d (%s)", detection.id, detection.metadata.name);
                telemetry.addData("Range", "%5.1f inches", detection.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", detection.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", detection.ftcPose.yaw);
                telemetry.addData("X", detection.center.x);
                telemetry.addData("Y", detection.center.y);
                telemetry.update();
            }
        }
    }

}
