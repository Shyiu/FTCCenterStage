package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class SlideTest extends LinearOpMode {

    DcMotor slide;

    @Override
    public void runOpMode() throws InterruptedException {
        slide = hardwareMap.get(DcMotor.class, "slides");
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            slide.setPower(-gamepad1.left_stick_y/2);
        }

        telemetry.addData("pos", slide.getCurrentPosition());

        telemetry.update();
    }
}
