package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name="Servo Testing")
@Config
public class ServoTest extends LinearOpMode {
    Servo servo;
    String name = "launcher_servo";
    ElapsedTime timer;
    double currentTime = 0;
    public static double position = 0;
    @Override
    public void runOpMode(){
        servo = hardwareMap.get(Servo.class, name);
        timer = new ElapsedTime();
        waitForStart();
        timer.reset();
        while(opModeIsActive() && !isStopRequested()){
            telemetry.addLine("a to add .1");
            telemetry.addLine("b to subtract .1");
            telemetry.addLine("x to add .01");
            telemetry.addLine("y to subtract .01");
            telemetry.addData("Servo Position: ", servo.getPosition());
            if (gamepad1.a && timer.time() - currentTime > .5){
                position += 0.1;
                currentTime= timer.time();
            }
            if (gamepad1.b && timer.time() - currentTime > .5){
                position -= 0.1;
                currentTime= timer.time();
            }
            if (gamepad1.x && timer.time() - currentTime > .5){
                position += 0.01;
                currentTime= timer.time();
            }
            if (gamepad1.y && timer.time() - currentTime > .5){
                position -= 0.01;
                currentTime= timer.time();
            }
            servo.setPosition(position);
            telemetry.update();
        }
    }
}
