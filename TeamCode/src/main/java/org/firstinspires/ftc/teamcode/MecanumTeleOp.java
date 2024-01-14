package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.autonomous_utilities.MathFunctions;
import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subclasses.Distance;
import org.firstinspires.ftc.teamcode.subclasses.MecaTank;
import org.firstinspires.ftc.teamcode.subclasses.PremPlaneLauncher;
import org.firstinspires.ftc.teamcode.subclasses.Rigging;
import org.firstinspires.ftc.teamcode.subclasses.RiggingOld;
import org.firstinspires.ftc.teamcode.subclasses.TempDelivery;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    private ElapsedTime time = new ElapsedTime();


//    VihasIntake arm;
    PremPlaneLauncher launcher;
    Rigging rigging;
    IMU imu;
    TempDelivery delivery;
    Distance distance;
    MecaTank mecatank;

    ElapsedTime timer = new ElapsedTime();


    public enum PLANE_STATE{
        LAUNCH,
        FLAT,
        WAIT
    }
    PLANE_STATE plane_state;

    public enum RIGGING_STATE {
        WAIT, EXTEND, RETRACT
    }

    RIGGING_STATE rigging_state;
    AprilTagPipeline apriltag_pipeline;
    ArrayList<Integer> apriltag_targets;

    public double planeTimer = 0;


    public MecanumBotConstant config = new MecanumBotConstant();



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        distance = new Distance(hardwareMap,telemetry);


        rigging_state = RIGGING_STATE.WAIT;
        plane_state = PLANE_STATE.WAIT;

        if(IMUTransfer.init) {
            imu = IMUTransfer.imu;

        }else{
            imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    DriveConstants.LOGO_FACING_DIR,
                    DriveConstants.USB_FACING_DIR));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
            IMUTransfer.imu = imu;
            IMUTransfer.init = true;
        }
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        apriltag_targets =  new ArrayList<>(Arrays.asList(1));

        mecatank = new MecaTank(hardwareMap, telemetry);

        launcher = new PremPlaneLauncher(hardwareMap);
        rigging = new Rigging(hardwareMap, telemetry);
        apriltag_pipeline = new AprilTagPipeline(hardwareMap);
        delivery = new TempDelivery(hardwareMap);


        delivery.setIn();





        // Makes the Driver Hub output the message "Status: Initialized"
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        time.reset();

        while (!isStopRequested() && opModeIsActive()) {

            mecatank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);


            switch (plane_state){
                case WAIT:
                    if (gamepad2.y){
                        launcher.setLaunchPosition();
                        plane_state = PLANE_STATE.LAUNCH;
                        planeTimer = timer.time();
//                        Pose2d start = calculatePose();

//                        if (start == null){
//                            running_auto = false;
//                            break;
//                        }else{
//                            running_auto = true;
//                        }
//                        running_auto = false;


                        break;
                    }
                    if(gamepad2.x){
                        launcher.setFlatPosition();
                        plane_state = PLANE_STATE.FLAT;
                        planeTimer = timer.time();
                        break;
                    }
                    break;
                case LAUNCH:
                    if(timer.time() - planeTimer > .300 && !drive.isBusy()){
                        launcher.launch();
                        plane_state = PLANE_STATE.WAIT;
                        break;
                    }
                    break;
                case FLAT:
                    if(timer.time() - planeTimer > .300){
                        plane_state = PLANE_STATE.WAIT;
                        break;
                    }
                    break;

            }
            Pose2d temp = calculatePose();
            try {
                telemetry.addData("Estimate X:", temp.getX());
                telemetry.addData("Estimate Y:", temp.getY());
                telemetry.addData("Estimate Heading:", temp.getHeading());
            }
            catch(NullPointerException e){
                telemetry.addLine("No April Tag Detected");
            }



            distance.telemetry();
            rigging.telemetry();
            mecatank.telemetry();

            telemetry.addData("Status", "Running");
            telemetry.update();
            drive.update();
        }

    }

    public Pose2d calculatePose(){
        Pose2d aprilTagPosition = new Pose2d();
        AprilTagDetection detection = apriltag_pipeline.getDetectionsForTargets(apriltag_targets);
        double range = 0;
        double degrees = 10;
        if (detection != null) {
            telemetry.addData("Target", "ID %d (%s)", detection.id, detection.metadata.name);
            telemetry.addData("Range", "%5.1f inches", detection.ftcPose.range);//35 in
            telemetry.addData("Bearing", "%3.0f degrees", detection.ftcPose.bearing);//12 degress
            telemetry.addData("Yaw", "%3.0f degrees", detection.ftcPose.yaw);
            telemetry.addData("X", detection.center.x);
            telemetry.addData("Y", detection.center.y);
            range = 35 - detection.ftcPose.range;
            degrees = detection.ftcPose.yaw;
            double y = range * Math.tan(Math.toRadians(degrees));
            aprilTagPosition = new Pose2d(range,y, Math.toRadians(degrees));
            telemetry.addData("Position", y);
            telemetry.addData("range", range);
            return aprilTagPosition;
        }
        return null;

    }
}


