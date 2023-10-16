package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Servo Testing")
@Config
public class ServoTest extends LinearOpMode {
    DcMotorSimple servo;
    String name = "test_servo";
    ElapsedTime timer;
    double currentTime = 0;
    @Override
    public void runOpMode(){
        servo = hardwareMap.get(DcMotorSimple.class, name);
        timer = new ElapsedTime();
        waitForStart();
        timer.reset();
        while(opModeIsActive() && !isStopRequested()){
            telemetry.addLine("a to add .1");
            telemetry.addLine("b to subtract .1");
            telemetry.addLine("x to add .01");
            telemetry.addLine("y to subtract .01");
            telemetry.addData("Servo Position: ", servo.getPower());
            if (gamepad1.a && timer.time() - currentTime > .5){
                servo.setPower(servo.getPower() + .1);
                currentTime= timer.time();
            }
            if (gamepad1.b && timer.time() - currentTime > .5){
                servo.setPower(servo.getPower() - .1);
                currentTime= timer.time();
            }
            if (gamepad1.x && timer.time() - currentTime > .5){
                servo.setPower(servo.getPower() + .01);
                currentTime= timer.time();
            }
            if (gamepad1.y && timer.time() - currentTime > .5){
                servo.setPower(servo.getPower() - .01);
                currentTime= timer.time();
            }
            telemetry.update();
        }
    }
}
