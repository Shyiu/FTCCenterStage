package org.firstinspires.ftc.teamcode.subclasses;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.pipelines.BoxDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.pipelines.BoxProcessor;

import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class DualVisionProcessor extends Subsystem{
    private VisionPortal boxVisonPortal, aprilTagVisionPortal;

    private BoxProcessor boxProcessor;
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;
    private MecanumBotConstant mc;
    public DualVisionProcessor(HardwareMap hardwareMap, BoxDetection boxDetection) throws InterruptedException {

        boxProcessor = new BoxProcessor(boxDetection);
        aprilTag = new AprilTagProcessor.Builder().build();
        mc = new MecanumBotConstant();
        boxVisonPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, mc.camera))
                    .setCameraResolution(new Size(960, 544))
                    .addProcessor(boxProcessor)
//                    .addProcessor(aprilTag)
                    .build();
        aprilTagVisionPortal= new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, mc.camera))
                .addProcessor(aprilTag)
                .build();
        setManualExposure(5, 200);
        aprilTagVisionPortal.stopStreaming();





    }
    public void activeAprilTag(){
        boxVisonPortal.close();
        aprilTagVisionPortal.resumeStreaming();
    }
    @Override
    public void telemetry() {

    }

    @Override
    public void init() {

    }

    public BoxDetection.Location getLocation(){
        return boxProcessor.getLocation();
    }
    public AprilTagDetection getDetectionsForTargets(ArrayList<Integer> targets){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) || (targets.indexOf(detection.id) > -1)){
                desiredTag = detection;
                return detection;
            }
        }
        return null;
    }

    public Pose2d getTargetPos(int tagid, double DESIRED_DISTANCE){
        ArrayList<Integer> targets  = new ArrayList<>();
        targets.add(tagid);
        AprilTagDetection detection = getDetectionsForTargets(targets);
        if(detection  == null){
            return new Pose2d(0,0,0);
        }

        double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        double  headingError    = desiredTag.ftcPose.bearing;
        double  yawError        = desiredTag.ftcPose.yaw;
        return new Pose2d(rangeError, -yawError, headingError);
    }
    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) throws InterruptedException {
        // Wait for the camera to be open, then use the controls

        if (aprilTagVisionPortal == null) {
            return;
        }
        // Make sure camera is streaming before we try to set the exposure controls
        if (aprilTagVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while ((aprilTagVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }

        // Set camera controls unless we are stopping.

        ExposureControl exposureControl = aprilTagVisionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);
        GainControl gainControl = aprilTagVisionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleep(20);
    }
}
