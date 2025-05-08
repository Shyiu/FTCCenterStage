package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subclasses.ShivaniRigging;

@Config
@TeleOp()
public class ContinuousServoTesting extends LinearOpMode {
    DcMotorSimple servo;
    public static double power = 0;


    @Override
    public void runOpMode(){
        servo = hardwareMap.get(DcMotorSimple.class, "test");

        waitForStart();


        while(!isStopRequested() && opModeIsActive()){
            servo.setPower(power == 0 ? -gamepad1.left_stick_y : power);
            telemetry.update();
        }
    }

}
