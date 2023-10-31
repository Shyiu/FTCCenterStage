package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subclasses.Encoder;

@Config
@TeleOp
public class DeadwheelDirectionDebug extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private BNO055IMU imu;
    private Encoder deadwheelLateral;
    private Encoder deadwheelLinear;
    @Override
    public void runOpMode() throws InterruptedException{
        MecanumBotConstant names = new MecanumBotConstant();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        frontRight = hardwareMap.get(DcMotor.class, names.fr);
        frontLeft = hardwareMap.get(DcMotor.class, names.fl);
        backRight = hardwareMap.get(DcMotor.class, names.br);
        backLeft = hardwareMap.get(DcMotor.class, names.bl);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        deadwheelLateral = new Encoder(hardwareMap, frontRight);
        deadwheelLinear = new Encoder(hardwareMap, backLeft);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            telemetry.addData("lateral", deadwheelLateral.getCurrentPosition());
            telemetry.addData("linear", deadwheelLinear.getCurrentPosition());
            telemetry.update();
        }

    }

    public double sameSignSqrt(double input){
        return Math.copySign(Math.sqrt(Math.abs(input)), input);
    }
}
