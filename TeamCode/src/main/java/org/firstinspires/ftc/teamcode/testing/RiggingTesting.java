package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subclasses.RiggingOld;

@Disabled
@TeleOp(name = "RiggingTest")
public class RiggingTesting extends LinearOpMode {
    DcMotor rigging_motor;
    @Override
    public void runOpMode(){
        rigging_motor = hardwareMap.get(DcMotor.class, "rigging");
            waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            rigging_motor.setPower(-gamepad1.left_stick_y);
            telemetry.update();
        }
    }

}
