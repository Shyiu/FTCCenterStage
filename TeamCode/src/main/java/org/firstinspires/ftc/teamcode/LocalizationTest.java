package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous_utilities.robot_utilities.Robot;
import org.firstinspires.ftc.teamcode.subclasses.Encoder;

@Config
@TeleOp
public class LocalizationTest extends LinearOpMode {
    Robot r;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        r = new Robot(hardwareMap);
        waitForStart();
        while(!isStopRequested() && opModeIsActive()){


            double leftpower = sameSignSqrt(-gamepad1.left_stick_y/2);
            double rightpower = sameSignSqrt(-gamepad1.right_stick_y/2);
            r.setMotorPowers(rightpower, leftpower);

            r.drawRobot();
            double[] powers = r.getPowers();
            r.updatePosition();
            telemetry.addData("X", r.getCurrentPosition()[0]);
            telemetry.addData("Y", r.getCurrentPosition()[1]);
            telemetry.addData("LeftPower", leftpower);
            telemetry.addData("RightPower", rightpower);
            telemetry.addData("frontLeft", powers[0]);
            telemetry.addData("frontRight", powers[1]);
            telemetry.addData("backLeft", powers[2]);
            telemetry.addData("backRight", powers[3]);
            telemetry.update();
        }

    }

    public double sameSignSqrt(double input){
        return Math.copySign(Math.sqrt(Math.abs(input)), input);
    }
}
