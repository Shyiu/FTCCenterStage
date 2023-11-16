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
import org.firstinspires.ftc.teamcode.pipelines.AprilTagPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subclasses.PlaneLauncher;
import org.firstinspires.ftc.teamcode.subclasses.Rigging;
import org.firstinspires.ftc.teamcode.subclasses.VihasIntake;
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

    VihasIntake arm;
    PlaneLauncher launcher;
    Rigging rigging;
    IMU imu;

    public boolean disableDrive = false;

    public static double MAX_SPEED = 1;
    public static double SERVO_SPEED = 1;

    ElapsedTime timer = new ElapsedTime();

    public enum DRIVE_STATE {
        DRIVE_TANK, DRIVE_STRAFE, FIELD_CENTRIC
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
    AprilTagPipeline pipeline;
    ArrayList<Integer> targets;

    public double servoTimer = 0;
    public double planeTimer = 0;
    public double servoDelay = .5;
    public boolean roller = false;

    double leftTgtPower = 0, rightTgtPower = 0;

    public MecanumBotConstant names = new MecanumBotConstant();


    public static DRIVE_STATE command = DRIVE_STATE.DRIVE_TANK;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        rigging_state = RIGGING_STATE.WAIT;
        plane_state = PLANE_STATE.WAIT;
        try{
            imu = IMUTransfer.imu;
        }catch(NullPointerException e){
            telemetry.addLine(e.toString());
            telemetry.update();
            imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    DriveConstants.LOGO_FACING_DIR,
                    DriveConstants.USB_FACING_DIR));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);

        }
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        targets =  new ArrayList<>(Arrays.asList(1));

        // Pulls the motors from the robot configuration so that they can be manipulated
        frontRight = hardwareMap.get(DcMotor.class, names.fr);
        frontLeft = hardwareMap.get(DcMotor.class, names.fl);
        backRight = hardwareMap.get(DcMotor.class, names.br);
        backLeft = hardwareMap.get(DcMotor.class, names.bl);

        launcher = new PlaneLauncher(hardwareMap);
        arm = new VihasIntake(hardwareMap);
        rigging = new Rigging(hardwareMap);
        pipeline = new AprilTagPipeline(hardwareMap);

        // Reverses the direction of the left motors, to allow a positive motor power to equal
        // forwards and a negative motor power to equal backwards
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);


        // Makes the Driver Hub output the message "Status: Initialized"
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        time.reset();

        while (!isStopRequested() && opModeIsActive()) {

            if (gamepad2.a) {
                arm.toggle();
            }
            switch (plane_state){
                case WAIT:
                    if (gamepad2.y){
                        launcher.setLaunchPos();
                        Pose2d start = calculatePose();
                        plane_state = PLANE_STATE.LAUNCH;
                        planeTimer = timer.time();
                        if (start == null){
                            running_auto = false;
                            break;
                        }else{
                            running_auto = true;
                        }
                        TrajectorySequence to_launch = drive.trajectorySequenceBuilder(start)
                                .lineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)))
                                .build();
                        drive.followTrajectorySequenceAsync(to_launch);

                        break;
                    }
                    if(gamepad2.x){
                        launcher.setFlat();
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
                    if(running_auto) {
                        drive.update();
                    }
                    break;
                case FLAT:
                    if(timer.time() - planeTimer > .300){
                        plane_state = PLANE_STATE.WAIT;
                        break;
                    }
                    break;

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
            rigging.setMotorLeftPower(-gamepad2.left_stick_y);
            rigging.setMotorRightPower(-gamepad2.right_stick_y);
            if (!disableDrive) {
                switch (command) {
                    case DRIVE_TANK:
                        double leftPower = sameSignSqrt(-gamepad1.left_stick_y);
                        double rightPower = sameSignSqrt(-gamepad1.right_stick_y);
                        frontLeft.setPower(leftPower * MAX_SPEED);
                        backLeft.setPower(leftPower * MAX_SPEED);
                        frontRight.setPower(rightPower * MAX_SPEED);
                        backRight.setPower(rightPower * MAX_SPEED);

                        if (leftPower == 0 && rightPower == 0) {
                            command = DRIVE_STATE.DRIVE_STRAFE;
                        }
                        telemetry.addLine("Drive mode: TANK");


                    case DRIVE_STRAFE:
                        if (gamepad1.left_trigger != 0) {
                            double backPower = sameSignSqrt(-gamepad1.left_trigger);
                            double frontPower = sameSignSqrt(gamepad1.left_trigger);
                            frontLeft.setPower(backPower * MAX_SPEED);
                            backRight.setPower(backPower * MAX_SPEED);
                            frontRight.setPower(frontPower * MAX_SPEED);
                            backLeft.setPower(frontPower * MAX_SPEED);

                        } else if (gamepad1.right_trigger != 0) {
                            double frontPower = sameSignSqrt(-gamepad1.right_trigger);
                            double backPower = sameSignSqrt(gamepad1.right_trigger);
                            frontLeft.setPower(backPower * MAX_SPEED);
                            backRight.setPower(backPower * MAX_SPEED);
                            frontRight.setPower(frontPower * MAX_SPEED);
                            backLeft.setPower(frontPower * MAX_SPEED);
                        } else {
                            command = DRIVE_STATE.DRIVE_TANK;
                        }
                        telemetry.addLine("Drive mode: STRAFE");
                        break;
                    case FIELD_CENTRIC:
                        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                        double x = gamepad1.left_stick_x;
                        double rx = gamepad1.right_stick_x;

                        // This button choice was made so that it is hard to hit on accident,
                        // it can be freely changed based on preference.
                        // The equivalent button is start on Xbox-style controllers.
                        if (gamepad1.options) {
                            imu.resetYaw();
                        }

                        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                        // Rotate the movement direction counter to the bot's rotation
                        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                        rotX = rotX * 1.1;  // Counteract imperfect strafing

                        // Denominator is the largest motor power (absolute value) or 1
                        // This ensures all the powers maintain the same ratio,
                        // but only if at least one is out of the range [-1, 1]
                        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                        double frontLeftPower = (rotY + rotX + rx) / denominator;
                        double backLeftPower = (rotY - rotX + rx) / denominator;
                        double frontRightPower = (rotY - rotX - rx) / denominator;
                        double backRightPower = (rotY + rotX - rx) / denominator;

                        frontLeft.setPower(frontLeftPower);
                        backLeft.setPower(backLeftPower);
                        frontRight.setPower(frontRightPower);
                        backRight.setPower(backRightPower);

                }
            }
            telemetry.addData("Left Target Power", leftTgtPower);
            telemetry.addData("Right Target Power", rightTgtPower);
            telemetry.addData("Front Right Motor Power", frontRight.getPower());
            telemetry.addData("Front Left Motor Power", frontLeft.getPower());
            telemetry.addData("Back Right Motor Power", backRight.getPower());
            telemetry.addData("Back Left Motor Power", backLeft.getPower());
            telemetry.addData("Rigging Position", rigging.getRiggingPosition());
//                telemetry.addData("Arm Power", arm.getArmPower());
            telemetry.addData("Status", "Running");

            telemetry.update();
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
        Pose2d output = new Pose2d();
        AprilTagDetection detection = pipeline.getDetectionsForTargets(targets);
        double range = 0;
        double degrees = 10;
        if (detection != null) {
            telemetry.addData("Target", "ID %d (%s)", detection.id, detection.metadata.name);
            telemetry.addData("Range", "%5.1f inches", detection.ftcPose.range);//35 in
            telemetry.addData("Bearing", "%3.0f degrees", detection.ftcPose.bearing);//12 degress
            telemetry.addData("Yaw", "%3.0f degrees", detection.ftcPose.yaw);
            telemetry.addData("X", detection.center.x);
            telemetry.addData("Y", detection.center.y);
            range = detection.ftcPose.range - 35;
            degrees = detection.ftcPose.bearing - 12;
            double y = range * Math.tan(Math.toRadians(degrees));
            output = new Pose2d(-range, -y, Math.toRadians(degrees));
            telemetry.addData("Position", y);
            telemetry.addData("range", range);
            return output;
        }
        return null;

    }
}


