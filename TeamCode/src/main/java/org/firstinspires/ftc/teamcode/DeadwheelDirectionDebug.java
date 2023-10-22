package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.Robot;
import org.firstinspires.ftc.teamcode.subclasses.Encoder;

@Config
@TeleOp
public class DeadwheelDirectionDebug extends LinearOpMode {

    private Orientation angles;
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
        deadwheelLinear = new Encoder(hardwareMap, frontLeft);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            telemetry.addData("lateral", deadwheelLateral.getCurrentPosition());
            telemetry.addData("liner", deadwheelLinear.getCurrentPosition());
            telemetry.update();
        }

    }

    public double sameSignSqrt(double input){
        return Math.copySign(Math.sqrt(Math.abs(input)), input);
    }
}
