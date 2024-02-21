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
import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    private ElapsedTime time = new ElapsedTime();

    IMU imu;
    Intake intake;
    MecaTank mecatank;
    MecanumDrive drive;
    ShivaniRigging shivaniRigging;
    PlaneLauncher planeLauncher;

    ElapsedTime timer = new ElapsedTime();


    public enum DELIVERY_STATE {
        WAIT,  INTAKE,PLUNGER_DOWN, TRANSFER, RETURN, DELIVERY, D1, RAISE_PLUNGER, D2
    }
    public enum THROW_STATE{
        WAIT, OPEN_PLUNGER, MOVE_ARM, RETURN_ARM
    }
    public enum RIGGING_STATE{
        WAIT, EXTEND, ADD_SLACK
    }
    THROW_STATE throw_state;
    DELIVERY_STATE delivery_state;
    RIGGING_STATE rigging_state;
    AprilTagPipeline apriltag_pipeline;
    ArrayList<Integer> apriltag_targets;

    public double delivery_timer = 0;
    public static int plane_launch_height = 260;
    public static boolean aarushi_being_useful = true;
    private boolean move_next = false;
    private boolean plane_launcher = false;
    public static boolean active_telemetry = false;
    private double plane_time = 0;
    private double throw_time = 0;
    private double rigging_time = 0;
    private boolean bucket_compensation = false;
    DELIVERY_STATE next_delivery_state = DELIVERY_STATE.INTAKE;

    public MecanumBotConstant config = new MecanumBotConstant();



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        delivery_state = DELIVERY_STATE.TRANSFER;
        throw_state = THROW_STATE.WAIT;
        rigging_state = RIGGING_STATE.WAIT;


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
        drive = new MecanumDrive(hardwareMap);


        mecatank = new MecaTank(hardwareMap, telemetry);

        intake = new Intake(hardwareMap, telemetry);

        planeLauncher = new PlaneLauncher(hardwareMap);

        shivaniRigging = new ShivaniRigging(hardwareMap, telemetry);

        mecatank.init();
        shivaniRigging.init();
        intake.init();
        planeLauncher.init();

        all_to_telemetry();


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        time.reset();

        while (!isStopRequested() && opModeIsActive()) {

            mecatank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_bumper ? 0 : gamepad1.left_trigger, gamepad1.right_trigger);





            switch(delivery_state){
                case WAIT:
                    break;
                case INTAKE:
                    intake.pickup();
                    delivery_state = DELIVERY_STATE.WAIT;
                    move_next = true;
                    break;
                case RAISE_PLUNGER:
                    intake.raise_clutch();
                    delivery_state = DELIVERY_STATE.WAIT;
                    break;
                case PLUNGER_DOWN:
                    intake.rotate_bucket();
                    delivery_state = DELIVERY_STATE.WAIT;
                    break;
                case TRANSFER:
                case RETURN:
                    intake.go_to_transfer();
                    delivery_state = DELIVERY_STATE.WAIT;
                    break;
                case DELIVERY:
                    intake.delivery();
                    delivery_state = DELIVERY_STATE.WAIT;
                    break;
                case D1:
                case D2:
                    intake.delivery_next();
                    delivery_state = DELIVERY_STATE.WAIT;

                    break;

            }
            intake.setPower(sameSignSqrt(-gamepad2.left_stick_y));

            boolean abutton = aarushi_being_useful ? gamepad2.a : gamepad1.a;
            if ((abutton || move_next) && time.time(TimeUnit.MILLISECONDS) - delivery_timer > 300 ){
                move_next = false;
                delivery_timer = time.time(TimeUnit.MILLISECONDS);
                switch(next_delivery_state){
                    case INTAKE:

                        delivery_state = DELIVERY_STATE.INTAKE;
                        next_delivery_state = DELIVERY_STATE.RAISE_PLUNGER;
                        break;
                    case RAISE_PLUNGER:
                        delivery_state = DELIVERY_STATE.RAISE_PLUNGER;
                        next_delivery_state = DELIVERY_STATE.PLUNGER_DOWN;
                        break;
                    case PLUNGER_DOWN:
                        delivery_state = DELIVERY_STATE.PLUNGER_DOWN;
                        next_delivery_state = DELIVERY_STATE.TRANSFER;
                        break;
                    case TRANSFER:
                        delivery_state = DELIVERY_STATE.TRANSFER;
                        next_delivery_state = DELIVERY_STATE.DELIVERY;
                        break;
                    case DELIVERY:
                        bucket_compensation = true;
                        delivery_state = DELIVERY_STATE.DELIVERY;
                        next_delivery_state = DELIVERY_STATE.D1;
                        break;
                    case D1:
                        delivery_state = DELIVERY_STATE.D1;
                        next_delivery_state = DELIVERY_STATE.D2;
                        break;
                    case D2:
                        delivery_state = DELIVERY_STATE.D2;
                        next_delivery_state = DELIVERY_STATE.RETURN;
                        break;
                    case RETURN:
                        bucket_compensation = false;
                        delivery_state = DELIVERY_STATE.RETURN;
                        next_delivery_state = DELIVERY_STATE.INTAKE;
                }
            }
            boolean bbutton = aarushi_being_useful ? gamepad2.b : gamepad1.b;

            if(bbutton){
                bucket_compensation = false;
                if(delivery_state == DELIVERY_STATE.DELIVERY){
                    delivery_state = DELIVERY_STATE.TRANSFER;
                    next_delivery_state = DELIVERY_STATE.DELIVERY;
                }else {
                    delivery_state = DELIVERY_STATE.INTAKE;
                    next_delivery_state = DELIVERY_STATE.RAISE_PLUNGER;
                }
            }
            switch(throw_state){
                case WAIT:








                    if(gamepad1.right_bumper) {
                        intake.moveClutch(0.25);
                        throw_state = THROW_STATE.OPEN_PLUNGER;
                        throw_time = timer.seconds();
                        break;
                    }
                    break;
                case OPEN_PLUNGER:
                    if (timer.seconds() - throw_time > 0.3){
                            intake.moveArm(2000);
                            throw_state = THROW_STATE.MOVE_ARM;
                            break;
                    }
                    break;
                case MOVE_ARM:
                    if(!intake.isBusy()){
                        intake.moveClutch(0);
                        intake.moveBucket(0);
                        intake.moveArm(0);
                        throw_state = THROW_STATE.RETURN_ARM;
                        break;
                    }
                    break;
                case RETURN_ARM:
                    if (!intake.isBusy()) {
                        throw_state = THROW_STATE.WAIT;
                        break;
                    }
                    break;
            }

            if(bucket_compensation){
                intake.bucket_compensation();
            }
            if(gamepad1.x){
                intake.moveArm(plane_launch_height);
                plane_launcher = true;
                plane_time = timer.seconds();
            }
            if(plane_launcher && !intake.isBusy() && timer.seconds() - plane_time > 0.4){
                planeLauncher.launch();
            }
//            switch(rigging_state){
//                case WAIT:
//                    if(gamepad2.x){
//                        shivaniRigging.openBoth();
//                        rigging_time = timer.time();
//                        rigging_state = RIGGING_STATE.EXTEND;
//                        break;
//                    }
//                    if(gamepad2.y){
//                        shivaniRigging.addSlack();
//                        rigging_state = RIGGING_STATE.ADD_SLACK;
//                        break;
//                    }
//                    break;
//                case EXTEND:
//                    if(timer.time() - rigging_time > .8){
//                        shivaniRigging.stop();
//                        rigging_state = RIGGING_STATE.WAIT;
//                        break;
//                    }
//                    break;
//                case ADD_SLACK:
//                    if(timer.time() - rigging_time > 0.1){
//                        shivaniRigging.stop();
//                        rigging_state = RIGGING_STATE.WAIT;
//                        break;
//                    }
//                    break;
//
//
//
//            }

            mecatank.set_min_distance(intake.calculate_robot_distance_limit(true));//sets the hardstop faster.
            shivaniRigging.setRiggingPower(sameSignSqrt(-gamepad2.right_stick_y));

            if(gamepad2.x){
                shivaniRigging.stop();
            }
            if(gamepad2.y){
                shivaniRigging.openBoth();
            }
            if(gamepad1.left_bumper){
                intake.addBucketPos(sameSignSqrt(gamepad1.left_trigger) / 4);
            }else{
                intake.addBucketPos(0);
            }



            all_to_telemetry();
            telemetry.addData("Status", "Running");
            telemetry.update();
            drive.update();
            intake.update();
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
        if(active_telemetry) {
            intake.telemetry();
            mecatank.telemetry();
            shivaniRigging.telemetry();
        }
    }
    public double sameSignSqrt(double number){
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }
}


