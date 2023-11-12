package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumBotConstant;
import org.firstinspires.ftc.teamcode.subclasses.Lift;
@Config
@TeleOp(name = "slide_testing")
public class SlideTest extends LinearOpMode {

    Lift lift;
    public static double P = 1;
    public static double I = 0;
    public static double D = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift = new Lift(hardwareMap, P ,I ,D);
        lift.reset();
        lift.moveTo(500);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()) {
            telemetry.addData("magnest", lift.getMagnet());
            lift.setPower(gamepad1.left_stick_y * -1);
//            lift.update();
            telemetry.addData("slide power", lift.getPower());
            telemetry.addData("slide test", lift.getCurrentPosition());
            telemetry.addData("slide other test", lift.exceedingConstraints());
            telemetry.addData("slide power", lift.getPower());

            telemetry.update();
        }
    }
}
