package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subclasses.Intake;

@Config
@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    private ElapsedTime time  = new ElapsedTime();
    protected DcMotor frontRight;
    protected DcMotor backRight;
    protected DcMotor frontLeft;
    protected DcMotor backLeft;

    Intake arm;

    public boolean disableDrive = false;

    public static double MAX_SPEED = 1;
    public static double SERVO_SPEED = 1;

    ElapsedTime timer = new ElapsedTime();
    public enum DRIVE_STATE{
        DRIVE_TANK,
        DRIVE_STRAFE
    }
    public double servoTimer = 0;
    public double servoDelay = .5;
    public boolean roller = false;

    double leftTgtPower = 0, rightTgtPower = 0;

    public MecanumBotConstant names = new MecanumBotConstant();




    public static DRIVE_STATE command = DRIVE_STATE.DRIVE_TANK;

    @Override
    public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            // Pulls the motors from the robot configuration so that they can be manipulated
            frontRight = hardwareMap.get(DcMotor.class, names.fr);
            frontLeft = hardwareMap.get(DcMotor.class, names.fl);
            backRight = hardwareMap.get(DcMotor.class, names.br);
            backLeft = hardwareMap.get(DcMotor.class, names.bl);


            arm = new Intake(hardwareMap);
            // Reverses the direction of the left motors, to allow a positive motor power to equal
            // forwards and a negative motor power to equal backwards
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.REVERSE);


            // Makes the Driver Hub output the message "Status: Initialized"
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            waitForStart();
            time.reset();

            while (!isStopRequested() && opModeIsActive()) {

                if(gamepad2.a){
                    arm.toggle();
                }
                arm.setPower(sameSignSqrt(gamepad2.left_stick_y/2));

                if(!disableDrive) {
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

                    }
                }

                telemetry.addData("Left Target Power", leftTgtPower);
                telemetry.addData("Right Target Power", rightTgtPower);
                telemetry.addData("Front Right Motor Power", frontRight.getPower());
                telemetry.addData("Front Left Motor Power", frontLeft.getPower());
                telemetry.addData("Back Right Motor Power", backRight.getPower());
                telemetry.addData("Back Left Motor Power", backLeft.getPower());
                telemetry.addData("Arm Power", arm.getArmPower());
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

        public double sameSignSqrt(double number){
            return Math.copySign(Math.sqrt(Math.abs(number)), number);
        }
}


