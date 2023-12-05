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
import org.firstinspires.ftc.teamcode.subclasses.PremPlaneLauncher;
import org.firstinspires.ftc.teamcode.subclasses.Rigging;
import org.firstinspires.ftc.teamcode.subclasses.TempDelivery;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    private ElapsedTime time = new ElapsedTime();
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;
    private boolean running_auto = false;

//    VihasIntake arm;
    PremPlaneLauncher launcher;
    Rigging rigging;
    IMU imu;
    TempDelivery delivery;
    Distance distance;

    public boolean disableDrive = false;

    public static double MAX_SPEED = .9;
    public static double SERVO_SPEED = 1;

    ElapsedTime timer = new ElapsedTime();

    public enum DRIVE_STATE {
        DRIVE_TANK, DRIVE_STRAFE, WAIT, SHAKE_PLANE, FIELD_CENTRIC
    }
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

    public double servoTimer = 0;
    public double planeTimer = 0;
    public double servoDelay = .5;
    public boolean roller = false;

    double left_target_power = 0, right_target_power = 0;

    public MecanumBotConstant config = new MecanumBotConstant();


    public static DRIVE_STATE command = DRIVE_STATE.DRIVE_TANK;

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

        // Pulls the motors from the robot configuration so that they can be manipulated
        frontRight = hardwareMap.get(DcMotor.class, config.fr);
        frontLeft = hardwareMap.get(DcMotor.class, config.fl);
        backRight = hardwareMap.get(DcMotor.class, config.br);
        backLeft = hardwareMap.get(DcMotor.class, config.bl);

        launcher = new PremPlaneLauncher(hardwareMap);
//        arm = new VihasIntake(hardwareMap);
        rigging = new Rigging(hardwareMap, telemetry);
        apriltag_pipeline = new AprilTagPipeline(hardwareMap);
        delivery = new TempDelivery(hardwareMap);
        delivery.setIn();

        // Reverses the direction of the left motors, to allow a positive motor power to equal
        // forwards and a negative motor power to equal backwards
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);



        // Makes the Driver Hub output the message "Status: Initialized"
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        time.reset();

        while (!isStopRequested() && opModeIsActive()) {
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
                        running_auto = false;


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

            //arm.setServoPower(sameSignSqrt(-gamepad2.left_stick_y));
//            switch (rigging_state) {
//                case WAIT:
//                    if (gamepad2.right_bumper) {
//                        rigging_state = RIGGING_STATE.EXTEND;
//                        rigging.rigUp();
//                        break;
//                    }
//                    if (gamepad2.left_bumper) {
//                        rigging_state = RIGGING_STATE.RETRACT;
//                        rigging.rigDown();
//                        break;
//                    }
//                case EXTEND:
//                    if (!rigging.isBusy()) {
//                        rigging_state = RIGGING_STATE.WAIT;
//                        break;
//                    }
//                    rigging.rigUp();
//
//                case RETRACT:
//                    if (!rigging.isBusy()) {
//                        rigging_state = RIGGING_STATE.WAIT;
//                        break;
//                    }
//                    rigging.rigDown();
//            }
            double gamepad2_left_y = -gamepad2.left_stick_y;
            double gamepad2_right_y = -gamepad2.right_stick_y;

            rigging.setMotorLeftPower(gamepad2_left_y);
            rigging.setMotorRightPower(gamepad2_right_y);

            if (!disableDrive) {
                switch (command) {
                    case DRIVE_TANK:
                        double leftPower = sameSignSqrt(-gamepad1.left_stick_y);
                        double rightPower = sameSignSqrt(-gamepad1.right_stick_y);

                        frontRight.setPower(leftPower * MAX_SPEED);
                        backRight.setPower(leftPower * MAX_SPEED);

                        frontLeft.setPower(rightPower * MAX_SPEED);
                        backLeft.setPower(rightPower * MAX_SPEED);

                        if (leftPower == 0 && rightPower == 0) {
                            command = DRIVE_STATE.DRIVE_STRAFE;
                        }
//                        if(gamepad1.a){
//                            command = DRIVE_STATE.SHAKE;
//                            break;
//                        }
                        telemetry.addLine("Drive mode: TANK");


                    case DRIVE_STRAFE:
                        if (gamepad1.left_trigger != 0) {
                            double posPower = sameSignSqrt(-gamepad1.left_trigger);
                            double negPower = sameSignSqrt(gamepad1.left_trigger);
                            frontLeft.setPower(posPower * MAX_SPEED);
                            backRight.setPower(posPower * MAX_SPEED);
                            frontRight.setPower(negPower * MAX_SPEED);
                            backLeft.setPower(negPower * MAX_SPEED);

                        } else if (gamepad1.right_trigger != 0) {
                            double posPower = sameSignSqrt(-gamepad1.right_trigger);
                            double negPower = sameSignSqrt(gamepad1.right_trigger);
                            frontLeft.setPower(negPower * MAX_SPEED);
                            backRight.setPower(negPower * MAX_SPEED);
                            frontRight.setPower(posPower * MAX_SPEED);
                            backLeft.setPower(posPower * MAX_SPEED);
                        } else {
                            command = DRIVE_STATE.DRIVE_TANK;
                        }
                        telemetry.addLine("Drive mode: STRAFE");
                        break;
                    case FIELD_CENTRIC:
                        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                        double x = gamepad1.left_stick_x;
                        double right_stick_x = gamepad1.right_stick_x;

                        // This button choice was made so that it is hard to hit on accident,
                        // it can be freely changed based on preference.
                        // The equivalent button is start on Xbox-style controllers.
                        if (gamepad1.options) {
                            imu.resetYaw();
                        }

                        double botHeading = MathFunctions.AngleWrap(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

                        // Rotate the movement direction counter to the bot's rotation
                        double rotationX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                        double rotationY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                        rotationX = rotationX * 1.1;  // Counteract imperfect strafing

                        // Denominator is the largest motor power (absolute value) or 1
                        // This ensures all the powers maintain the same ratio,
                        // but only if at least one is out of the range [-1, 1]
                        double denominator = Math.max(Math.abs(rotationY) + Math.abs(rotationX) + Math.abs(right_stick_x), 1);
                        double frontLeftPower = (rotationY + rotationX + right_stick_x) / denominator;
                        double backLeftPower = (rotationY - rotationX + right_stick_x) / denominator;
                        double frontRightPower = (rotationY - rotationX - right_stick_x) / denominator;
                        double backRightPower = (rotationY + rotationX - right_stick_x) / denominator;

                        frontLeft.setPower(frontLeftPower);
                        backLeft.setPower(backLeftPower);
                        frontRight.setPower(frontRightPower);
                        backRight.setPower(backRightPower);
                        break;
                    case SHAKE_PLANE:
                        if(!drive.isBusy()) {
                            double distance_to_backdrop = distance.getDist();
                            if (distance_to_backdrop > 75) {
                                distance_to_backdrop = 0;
                            } else {
                                distance_to_backdrop -= 26;
                            }

                            TrajectorySequence to_launch_position = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                    .back(6)
                                    .forward(6)
                                    .build();
                            drive.setPoseEstimate(new Pose2d());
                            drive.followTrajectorySequenceAsync(to_launch_position);
                            command = DRIVE_STATE.WAIT;
                        }
                        break;
                    case WAIT:
                        if(!drive.isBusy()){
                            command = DRIVE_STATE.DRIVE_TANK;
                            break;
                        }
                        break;



                }
            }


            distance.telemetry();
            rigging.telemetry();

            telemetry.addData("Left Target Power", left_target_power);
            telemetry.addData("Right Target Power", right_target_power);
            telemetry.addData("Front Right Motor Power", frontRight.getPower());
            telemetry.addData("Front Left Motor Power", frontLeft.getPower());
            telemetry.addData("Back Right Motor Power", backRight.getPower());
            telemetry.addData("Back Left Motor Power", backLeft.getPower());



//                telemetry.addData("Arm Power", arm.getArmPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
            drive.update();
        }

    }


//        public double closerToV2(double v1, double v2, double v3){
//            double diff1 = Math.abs(v1-v2);
//            double diff2 = Math.abs(v2-v3);
//            if (diff1 > diff2){
//                return v1;
//            }
//            return v3;
//        }

    public double sameSignSqrt(double number) {
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }
    public boolean inRange(double number, double target) {
        double max = target + 0.5;
        double min = target - 0.5;
        return (number <= max) && (number >= min);
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


