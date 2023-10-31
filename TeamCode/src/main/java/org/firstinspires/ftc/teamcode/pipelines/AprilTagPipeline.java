package org.firstinspires.ftc.teamcode.pipelines;



//import com.acmerobotics.dashboard.config.Config;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

//for dashboard
//@Config
public class AprilTagPipeline {
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;
    private MecanumBotConstant mc = new MecanumBotConstant();
    private boolean timer_toggle = true;
    private int ms = 0;
    private ElapsedTime timer;
    /**
     * Initialize the AprilTag processor.
     */
    public AprilTagPipeline(HardwareMap hardwareMap) throws InterruptedException {
        // Create the AprilTag processor by using a builder.
        timer = new ElapsedTime();
        aprilTag = new AprilTagProcessor.Builder().build();
        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, mc.camera))
                    .addProcessor(aprilTag)
                    .build();
        setManualExposure(5, 200);

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
    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) throws InterruptedException {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }
        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
       }

        // Set camera controls unless we are stopping.

            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
    }
}