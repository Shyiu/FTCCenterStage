package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Arrays;

public class PipelineTesting extends LinearOpMode {
    AprilTagPipeline pipeline;
    ArrayList<Integer> targets;
    Telemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        pipeline.initAprilTag(hardwareMap);
        targets =  new ArrayList<>(Arrays.asList(1,2,3,4,5,6));
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            AprilTagDetection detection = pipeline.getDetectionsForTargets(targets);
            telemetry.addData("Target", "ID %d (%s)", detection.id, detection.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", detection.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", detection.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", detection.ftcPose.yaw);
            telemetry.addData("X", detection.center.x);
            telemetry.addData("Y",detection.center.y);
            telemetry.update();
        }
    }

}
