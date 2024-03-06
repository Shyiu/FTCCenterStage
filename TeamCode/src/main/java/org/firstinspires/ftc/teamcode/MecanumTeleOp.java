package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subclasses.Distance;
import org.firstinspires.ftc.teamcode.subclasses.Intake;
import org.firstinspires.ftc.teamcode.subclasses.MecaTank;
import org.firstinspires.ftc.teamcode.subclasses.PlaneLauncher;
import org.firstinspires.ftc.teamcode.subclasses.ShivaniRigging;
import org.firstinspires.ftc.teamcode.subclasses.Unicorn;
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
    Unicorn unicorn;
    ShivaniRigging shivaniRigging;
    PlaneLauncher planeLauncher;
    Distance distance_back;
    Distance distance_front;
    ElapsedTime timer = new ElapsedTime();


    public enum DELIVERY_STATE {
        WAIT,  INTAKE,PLUNGER_DOWN, TRANSFER, RETURN, DELIVERY, D1, RAISE_PLUNGER, D2, START_THROW, MOVE_ARM_THROW, BACK_UP_DELAY, RETURN_ARM_THROW
    }

    public enum PLANE_STATE{
        WAIT, MOVE, LAUNCH
    }
    public enum SCANNING_STATE{
        WAIT, STRAFE, FIRST_HIT, LAST_HIT, STRAFE_ADJUSTMENT, RESET, APPROACH, START_MOVE, BEGIN_STRAFE, INTAKE
    }
    public enum UNICORN_STATE{
        WAIT, DELIVER, RETRACT,EXTEND
    }
    DELIVERY_STATE delivery_state;
    UNICORN_STATE unicorn_state;
    PLANE_STATE plane_state;
    AprilTagPipeline apriltag_pipeline;
    ArrayList<Integer> apriltag_targets;
    SCANNING_STATE scanning_state;

    public double delivery_timer = 0;
    public static int plane_launch_height = 210;
    public static double PIXEL_DISTANCE = 3.3;
    public static boolean aarushi_being_useful = true;
    private boolean move_next = false;
    private boolean skip_auto_alignment = false;
    private boolean lock_stow = false;
    private boolean scanning = false;
    private boolean disabled_zero = false;
    public static boolean active_telemetry = false;
    public static boolean red = false;
    private double plane_time = 0;
    private double throw_time = 0;
    private double rigging_time = 0;
    private double unicorn_time = 0;
    private double scanning_time;
    private double delay_timer = 0;
    private boolean rr_on = false;
    private double past_distance_reading = 0;
    private double past_distance_timer = 0;
    private double starting_position = 0;
    public static double BACKDROP_DISTANCE = 18.00;
    public static double PIXEL_TIME = 0.6;
    private boolean bucket_compensation = false;
    private boolean max_speed_change = false;
    public static double CORRECTION_HEADING_BACK = -0.05;
    public static double CORRECTION_HEADING_FORWARD = 0.05;
    private double MAX_SPEED = 1;
    DELIVERY_STATE next_delivery_state = DELIVERY_STATE.INTAKE;

    public MecanumBotConstant config = new MecanumBotConstant();



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        delivery_state = DELIVERY_STATE.TRANSFER;

        unicorn_state = UNICORN_STATE.WAIT;

        plane_state = PLANE_STATE.WAIT;

        scanning_state = SCANNING_STATE.WAIT;

        if(DataTransfer.init) {
            imu = DataTransfer.imu;
        }else{
            imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    DriveConstants.LOGO_FACING_DIR,
                    DriveConstants.USB_FACING_DIR));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
            DataTransfer.imu = imu;
            DataTransfer.init = true;
        }
//        red = DataTransfer.red;
        drive = new MecanumDrive(hardwareMap);


        mecatank = new MecaTank(hardwareMap, telemetry);

        intake = new Intake(hardwareMap, telemetry);

        planeLauncher = new PlaneLauncher(hardwareMap);

        shivaniRigging = new ShivaniRigging(hardwareMap, telemetry);

        unicorn = new Unicorn(hardwareMap, telemetry);

        distance_back = new Distance(hardwareMap, telemetry);

        distance_front = new Distance(hardwareMap, telemetry, true);


        mecatank.init();
        shivaniRigging.init();
        intake.init();
        intake.setSlowPower(0.5);
        intake.setSlowPosition(1650);

        planeLauncher.init();

        distance_back.init();


        all_to_telemetry();


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        time.reset();

        while (!isStopRequested() && opModeIsActive()) {

            if(timer.time() - past_distance_timer > 0.005){
                past_distance_reading = distance_front.getDist();
                past_distance_timer = timer.time();
            }

            if(gamepad1.dpad_left){
                if(!rr_on || max_speed_change) {
                    drive.setWeightedDrivePower(new Pose2d(-MAX_SPEED, 0, CORRECTION_HEADING_BACK));
                    max_speed_change =false;
                }
                rr_on = true;
            }else if(gamepad1.dpad_right){
                if(!rr_on || max_speed_change) {
                    drive.setWeightedDrivePower(new Pose2d(MAX_SPEED, 0, CORRECTION_HEADING_FORWARD));
                    max_speed_change = false;
                }
                rr_on = true;
            }else if(rr_on) {
                rr_on = false;
                drive.setWeightedDrivePower(new Pose2d());
            }else if(!drive.isBusy() && !scanning){
                mecatank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_bumper ? 0 : gamepad1.left_trigger, gamepad1.right_trigger);
            }
            if(gamepad1.y && scanning && !gamepad1.left_bumper){
                scanning = false;
                scanning_state = SCANNING_STATE.RESET;
                scanning_time = timer.time();
                drive.setWeightedDrivePower(new Pose2d());
            }

            if(gamepad2.right_bumper){
                intake.setPower(0);
                intake.moveArm(intake.getPosition());
                shivaniRigging.setHookPower(0);
                shivaniRigging.moveHook(shivaniRigging.getHookPosition());
            }
            switch(delivery_state){
                case WAIT:
                    if(gamepad1.right_bumper) {
                        intake.raise_clutch();
                        delivery_state = DELIVERY_STATE.START_THROW;
                        throw_time = timer.seconds();

                        break;
                    }
                    break;
                case START_THROW:
                    if (timer.seconds() - throw_time > 0.1){
                        intake.moveArm(200);
                        delivery_state = DELIVERY_STATE.MOVE_ARM_THROW;
                        break;
                    }
                    break;
                case MOVE_ARM_THROW:
                    if(!intake.isBusy()){
                        intake.moveBucket(.9);
                        throw_time = timer.time();
                        delivery_state = DELIVERY_STATE.RETURN_ARM_THROW;
                        break;
                    }
                    break;
                case RETURN_ARM_THROW:
                    if (timer.seconds() - throw_time > 0.2) {

                        delivery_state = DELIVERY_STATE.TRANSFER;
                        next_delivery_state = DELIVERY_STATE.INTAKE;
                        break;
                    }
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
                    delay_timer = timer.time();
                    drive.setWeightedDrivePower(new Pose2d(-0.5,0,0));
                    move_next = true;
                    break;
                case BACK_UP_DELAY:
                    if(timer.time() - delay_timer > 0.3){
                        drive.setWeightedDrivePower(new Pose2d());
                        delivery_state = DELIVERY_STATE.WAIT;
                        move_next = true;
                    }
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
                    Pose2d drive_estimate = drive.getPoseEstimate();
                    if(distance_back.getDistFromRobotEdge() < 100) {
                        drive.setPoseEstimate(new Pose2d(distance_back.getDistFromRobotEdge(), drive_estimate.getY(), drive_estimate.getHeading()));
                    }
                    break;

            }
            double intake_power = sameSignSqrt(gamepad2.left_stick_y/2.0);
            if((distance_back.getDistFromRobotEdge() < intake.calculate_robot_distance_limit(true) + 1) && intake_power > 0 && intake.getPosition() > intake.calculate_arm_limit(distance_back.getDistFromRobotEdge()) ){
                intake.setPower(0);
            }else{
                intake.setPower(intake_power);
            }
            if(gamepad2.left_bumper){
                intake.setPower(-0.005);
            }
            if(gamepad2.y && timer.time() > 90){
                shivaniRigging.release_motor();
            }
            double target_position = 7;//distance sensor in front - offset (pixel length)
            switch(scanning_state){
                case RESET:
                    if(timer.time() - scanning_time > 0.3){
                        scanning_state=  SCANNING_STATE.WAIT;
                    }
                    break;
                case WAIT:
                    scanning = false;
                    if(!gamepad1.left_bumper && gamepad1.y){
                        intake.moveBucket(0.48);
                        scanning_state = SCANNING_STATE.START_MOVE;
                        scanning_time = timer.time();
                        scanning =true;
                    }if(gamepad1.left_bumper && gamepad1.y){
                        intake.moveBucket(0.53);
                        scanning_state = SCANNING_STATE.BEGIN_STRAFE;
                        scanning_time = timer.time() + 0.3;
                        scanning = true;
                    }
                    break;
                case START_MOVE:
                    if(timer.time() - scanning_time > 0.4) {
                        starting_position = distance_front.getDist();
                        if(starting_position > 30){
                            scanning_state = SCANNING_STATE.WAIT;
                            break;
                        }
                        Trajectory toDistance = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .forward(starting_position - target_position ,
                                        drive.getVelocityConstraint(DriveConstants.MAX_VEL * .40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        drive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.40))
                                .build();
                        drive.followTrajectoryAsync(toDistance);
                        scanning_state = SCANNING_STATE.APPROACH;
                        scanning = true;
                    }
                    break;

                case APPROACH:
                    if(!drive.isBusy()) {
                        drive.setWeightedDrivePower(new Pose2d());
                        scanning_time = timer.time();
                        scanning_state = SCANNING_STATE.BEGIN_STRAFE;
                        intake.moveBucket(0.53);
                    }

                    break;
                case BEGIN_STRAFE:
                    if(timer.time() - scanning_time > 0.3){
                        starting_position = drive.getPoseEstimate().getY();

                        scanning_time = timer.time();
                        drive.setWeightedDrivePower(new Pose2d(0, red ? 0.3 : -0.3, 0));
                        scanning_state = SCANNING_STATE.FIRST_HIT;
                    }
                    break;

                case FIRST_HIT:
                    if(Math.abs(drive.getPoseEstimate().getY() - starting_position) > 15){
                        scanning_state = SCANNING_STATE.WAIT;
                        drive.setDrivePower(new Pose2d());
                        break;

                    }
                    if(Math.abs(past_distance_reading - distance_front.getDist()) > 0.8){//Change to distance sensor picks up a drop instead.
                        intake.pickup();

                        if (red) {
                            drive.setWeightedDrivePower(new Pose2d(0, -0.2, 0));
                        }
                        scanning_state = SCANNING_STATE.STRAFE;
                        scanning_time = timer.time();
                        starting_position = drive.getPoseEstimate().getY();
                    }
                    break;
                    //Check if the color sensor detects less than 25 cm or something.
                case STRAFE:
                    if((red && timer.time() - scanning_time > PIXEL_TIME) || (!red && Math.abs(drive.getPoseEstimate().getY() - starting_position) > PIXEL_DISTANCE)){
                        drive.setWeightedDrivePower(new Pose2d());
                        scanning_state = SCANNING_STATE.INTAKE;
                    }
                    break;
                //strafe for (x) seconds after first hit.
                case INTAKE:
                    next_delivery_state = DELIVERY_STATE.INTAKE;
                    move_next = true;
                    scanning_state = SCANNING_STATE.WAIT;
                    break;


            }
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
                        next_delivery_state = DELIVERY_STATE.BACK_UP_DELAY;
                        break;
                    case BACK_UP_DELAY:
                        delivery_state = DELIVERY_STATE.BACK_UP_DELAY;
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
            switch(unicorn_state){
                case WAIT:
                    if(gamepad1.dpad_up && !DataTransfer.delivered ){
                        unicorn_state = UNICORN_STATE.DELIVER;
                        unicorn.deliver();
                        unicorn_time = timer.time();
                    }
                    break;
                case DELIVER:
                    if(timer.time() - unicorn_time > 0.3){
                        unicorn.stow();
                        unicorn_time = timer.time();
                        unicorn_state  = UNICORN_STATE.RETRACT;
                    }
                    break;
                case RETRACT:
                    if(timer.time() - unicorn_time > 0.3){
                        unicorn_state = UNICORN_STATE.WAIT;
                        DataTransfer.delivered = true;
                    }
                    break;
                case EXTEND:
                    if(timer.time() - unicorn_time > 0.3){
                        unicorn_state = UNICORN_STATE.WAIT;
                    }
                    break;
            }

            if(bucket_compensation){
                intake.bucket_compensation();
            }
            switch(plane_state){
                case WAIT:
                    if(gamepad1.left_bumper && gamepad1.x){
                        intake.moveBucket(0.48);
                        plane_time = timer.seconds();
                        plane_state = PLANE_STATE.MOVE;
                        skip_auto_alignment = false;
                    }else if(!gamepad1.left_bumper && gamepad1.x){
                        plane_state = PLANE_STATE.MOVE;
                        plane_time = timer.seconds() - .5;
                        skip_auto_alignment = true;
                    }
                    break;
                case MOVE:
                    intake.moveArm(plane_launch_height);

                    Pose2d start = drive.getPoseEstimate();
                    double distance = distance_front.getDist();
                    if(distance > 30 || skip_auto_alignment){
                        plane_time = timer.seconds();
                        plane_state = PLANE_STATE.LAUNCH;
                        break;
                    }
                    Trajectory toLaunch = drive.trajectoryBuilder(start)
                            .forward(distance_front.getDist() - BACKDROP_DISTANCE)
                            .build();
                    drive.followTrajectoryAsync(toLaunch);
                case LAUNCH:
                    if(!drive.isBusy() && !intake.isCompleteFor(2) && timer.seconds() - plane_time > 0.4){
                        planeLauncher.launch();
                    }
                    break;
            }



            if(gamepad1.a) {
                max_speed_change = MAX_SPEED == 0.4;
                mecatank.setMaxPower(0.4);
                MAX_SPEED = 0.4;
                mecatank.set_min_distance(0);
                drive.breakFollowing();//sets the hardstop faster.

            }
            else if(distance_back.getDistFromRobotEdge() < 24 && (next_delivery_state == DELIVERY_STATE.D1) ){
                double speed = (24 - Math.max(0,24 - distance_back.getDistFromRobotEdge()))/(24.0);
                speed = Math.max(0.4, speed);
                max_speed_change = MAX_SPEED == speed;

                mecatank.set_min_distance(intake.calculate_robot_distance_limit(true));
                MAX_SPEED = speed;
                mecatank.setMaxPower(speed);
            }
            else{
                max_speed_change = MAX_SPEED == 1;

                mecatank.set_min_distance(intake.calculate_robot_distance_limit(true));
                mecatank.setMaxPower(1);
                MAX_SPEED = 1;

            }
            double rigging_power = sameSignSqrt(gamepad2.right_stick_y);
            if(rigging_power == 0 && !lock_stow && DataTransfer.delivered){
                unicorn.stow();
            }else if(timer.time() > 85){
                if(!lock_stow) {
                    unicorn_time = timer.time();
                }
                lock_stow = true;
                unicorn.rigging();
            }
            if(lock_stow && timer.time() - unicorn_time > 0.3) {
                shivaniRigging.setRiggingPower(rigging_power);
            }
            else{
                shivaniRigging.setRiggingPower(0);
            }

            if(gamepad1.left_bumper){
                intake.addBucketPos(sameSignSqrt(gamepad1.left_trigger) / 4);
            }else{
                intake.addBucketPos(0);
            }
            if(gamepad2.x && timer.time() > 85){
                shivaniRigging.raise_hooks_to_sensor();
                disabled_zero = true;
            }else if(!gamepad1.left_bumper && gamepad1.dpad_down){
                shivaniRigging.setHookPower(-.3);
            }else if(gamepad1.dpad_down && gamepad1.left_bumper){
                shivaniRigging.setHookPower(.4);
            }
            else{
                if(!disabled_zero) {
                    shivaniRigging.setHookPower(0);
                }
            }


            all_to_telemetry();
//            distance_front.telemetry();
            telemetry.addData("Status", "Running");
            telemetry.update();
            drive.update();
            intake.update();
            if(disabled_zero) {
                shivaniRigging.update();
            }

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
            telemetry.addData("Bearing", "%3.0f degrees", detection.ftcPose.bearing);// degress
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


