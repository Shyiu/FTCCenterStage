package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subclasses.Intake;
import org.firstinspires.ftc.teamcode.subclasses.MecaTank;
import org.firstinspires.ftc.teamcode.subclasses.PlaneLauncher;
import org.firstinspires.ftc.teamcode.subclasses.ShivaniRigging;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    private ElapsedTime time = new ElapsedTime();

    PlaneLauncher launcher;
    ShivaniRigging rigging;
    IMU imu;
    Intake intake;
    MecaTank mecatank;

    ElapsedTime timer = new ElapsedTime();

    public enum PLANE_STATE{
        LAUNCH,
        FLAT,
        WAIT
    }

    PLANE_STATE plane_state;

    public enum DELIVERY_STATE {
        WAIT, 
        INTAKE, 
        TRANSFER, 
        DELIVERY
    }

    DELIVERY_STATE delivery_state;
    AprilTagPipeline apriltag_pipeline;
    ArrayList<Integer> apriltag_targets;

    public double delivery_timer = 0;
    DELIVERY_STATE next_delivery_state = DELIVERY_STATE.INTAKE;

    public MecanumBotConstant config = new MecanumBotConstant();



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        delivery_state = DELIVERY_STATE.TRANSFER;
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

        launcher = new PlaneLauncher(hardwareMap);
        rigging = new ShivaniRigging(hardwareMap, telemetry);
        apriltag_pipeline = new AprilTagPipeline(hardwareMap);
        intake = new Intake(hardwareMap, telemetry);


        mecatank.init();
        launcher.init();
        rigging.init();
        intake.init();

        all_to_telemetry();


        // Makes the Driver Hub output the message "Status: Initialized"
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        time.reset();

        while (!isStopRequested() && opModeIsActive()) {

            mecatank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);


            switch (plane_state){
               case WAIT:
                   if (gamepad1.x){
                       launcher.setLaunchPosition();
                       plane_state = PLANE_STATE.LAUNCH;
                       planeTimer = timer.time();
                       break;
                   }
               case LAUNCH:
                   if(timer.time() - planeTimer > .600 && !drive.isBusy()){
                       launcher.launch();
                       rigging.activate();
                       plane_state = PLANE_STATE.WAIT;
                       break;
                   }
                   break;

               

           }

            switch(delivery_state){
                case WAIT:
                    break;
                case INTAKE:
                    intake.pickup();
                    delivery_state = DELIVERY_STATE.WAIT;
                    break;
                case TRANSFER:
                    intake.go_to_transfer();
                    delivery_state = DELIVERY_STATE.WAIT;

                    break;
                case DELIVERY:
                    intake.delivery();
                    delivery_state = DELIVERY_STATE.WAIT;


                    break;
            }

            intake.increaseRotation(-gamepad2.left_stick_y/6.0);
            intake.slides.setPower(-gamepad2.right_stick_y/2.0);

            if (gamepad2.a && time.time(TimeUnit.MILLISECONDS) - delivery_timer > 300){
                delivery_timer = time.time(TimeUnit.MILLISECONDS);
                switch(next_delivery_state){
                    case INTAKE:
                        delivery_state = DELIVERY_STATE.INTAKE;
                        next_delivery_state = DELIVERY_STATE.TRANSFER;
                        break;
                    case TRANSFER:
                        delivery_state = DELIVERY_STATE.TRANSFER;
                        next_delivery_state = DELIVERY_STATE.DELIVERY;
                        break;
                    case DELIVERY:
                        delivery_state = DELIVERY_STATE.DELIVERY;
                        next_delivery_state = DELIVERY_STATE.INTAKE;
                        break;
                }
            }
            if (gamepad2.x){
                intake.enable_rollers();
            }else{
                intake.disable_rollers();
            }


            intake.update();
            rigging.update();
            all_to_telemetry();
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
    public void all_to_telemetry(){
        intake.telemetry();
        launcher.telemetry();
        rigging.telemetry();
        mecatank.telemetry();
    }
}


