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
import org.firstinspires.ftc.teamcode.subclasses.MecaTank;

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
    public MecaTank mecatank;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        MecanumDrive drive = new MecanumDrive(hardwareMap);

        mecatank = new MecaTank(hardwareMap, telemetry);


        mecatank.init();
        // Makes the Driver Hub output the message "Status: Initialized"
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        time.reset();

        while (!isStopRequested() && opModeIsActive()) {

            mecatank.setPowers(gamepad1.left_stick_y, gamepad1.right_stick_y, gamepad1.left_trigger, gamepad1.right_trigger);

            mecatank.telemetry();
            drive.update();
            telemetry.update();
        }


//        public double closerToV2(double v1, double v2, double v3){
//            double diff1 = Math.abs(v1-v2);
//            double diff2 = Math.abs(v2-v3);
//            if (diff1 > diff2){
//                return v1;
//            }
//            return v3;
//        }
    }
    public double sameSignSqrt(double number) {
        return Math.copySign(Math.sqrt(Math.abs(number)), number);
    }
    public boolean inRange(double number, double target) {
        double max = target + 0.5;
        double min = target - 0.5;
        return (number <= max) && (number >= min);
    }
}


