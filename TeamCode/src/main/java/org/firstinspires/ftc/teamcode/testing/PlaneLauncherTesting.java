package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Plane Launcher Testing")
@Config
public class PlaneLauncherTesting extends LinearOpMode {
    DcMotorSimple servo;
    String name = "test_servo";
    ElapsedTime timer;
    double currentTime = 0;
    double primed = -1;
    double launch = 0;
    @Override
    public void runOpMode(){
        servo = hardwareMap.get(DcMotorSimple.class, name);
        timer = new ElapsedTime();
        waitForStart();
        timer.reset();
        while(opModeIsActive() && !isStopRequested()){
            telemetry.addLine("a to prime");
            telemetry.addLine("b to launch");

            telemetry.addData("Servo Position: ", servo.getPower());
            if (gamepad1.a && timer.time() - currentTime > .5){
                servo.setPower(primed);
                currentTime= timer.time();
            }
            if (gamepad1.b && timer.time() - currentTime > .5){
                servo.setPower(launch);
                currentTime= timer.time();
            }

            telemetry.update();
        }
    }
}
