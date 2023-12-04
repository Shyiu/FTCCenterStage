package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.roadrunner.drive.MecanumDrive;

@Config
@TeleOp
public class DriveTrainTesting extends LinearOpMode {
    private ElapsedTime time = new ElapsedTime();
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;


    public static double MAX_SPEED = .9;

    ElapsedTime timer = new ElapsedTime();

    public enum DRIVE_STATE {
        DRIVE_TANK, DRIVE_STRAFE, WAIT, SHAKE, FIELD_CENTRIC
    }



    double leftTgtPower = 0, rightTgtPower = 0;

    public MecanumBotConstant names = new MecanumBotConstant();


    public static DRIVE_STATE command = DRIVE_STATE.DRIVE_TANK;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());





        MecanumDrive drive = new MecanumDrive(hardwareMap);


        // Pulls the motors from the robot configuration so that they can be manipulated
        frontRight = hardwareMap.get(DcMotor.class, names.fr);
        frontLeft = hardwareMap.get(DcMotor.class, names.fl);
        backRight = hardwareMap.get(DcMotor.class, names.br);
        backLeft = hardwareMap.get(DcMotor.class, names.bl);


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
                }
            }


            telemetry.addData("Left Target Power", leftTgtPower);
            telemetry.addData("Right Target Power", rightTgtPower);
            telemetry.addData("Front Right Motor Power", frontRight.getPower());
            telemetry.addData("Front Left Motor Power", frontLeft.getPower());
            telemetry.addData("Back Right Motor Power", backRight.getPower());
            telemetry.addData("Back Left Motor Power", backLeft.getPower());



//                telemetry.addData("Arm Power", arm.getArmPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
            drive.update();
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
}


